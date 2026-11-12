"""Check (and fix) EFeru Inc/config.h for the VARIANT_USART build we flash.

config.h is a pile of #ifdef VARIANT_* blocks, some of which #undef and
redefine global settings (VARIANT_HOVERCAR redefines CTRL_MOD_REQ, for one).
Grepping for the first `#define CTRL_MOD_REQ` would answer for the wrong
variant, so this runs a small preprocessor over the file with the macros the
VARIANT_USART env defines, and reports the values that build would actually see
— together with the line each one came from, which is the line a fix edits.

Only what config.h needs is supported: #if/#ifdef/#ifndef/#elif/#else/#endif,
#define/#undef, defined(), ! && || == != < > <= >=, integer literals, line
continuations. #include is ignored; a macro from a header is unknown (0).
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Callable

ENV = 'VARIANT_USART'
FUSE_A = 20                    # the 20 A / 60 V fuse on the battery line
RECOMMENDED_I_MOT_MAX = 8      # 2 * (8 + 2) = 20 A total, never above the fuse
RECOMMENDED_I_DC_MAX = 10


@dataclass
class Macro:
    value: str
    line: int                  # 0-based line index in the file


@dataclass
class Preprocessed:
    macros: dict[str, Macro]
    warnings: list[str] = field(default_factory=list)


def _logical_lines(text: str):
    """(first_line_index, joined_text) with backslash continuations folded."""
    lines = text.split('\n')
    i = 0
    while i < len(lines):
        start, buf = i, lines[i]
        while buf.rstrip().endswith('\\') and i + 1 < len(lines):
            i += 1
            buf = buf.rstrip()[:-1] + ' ' + lines[i]
        yield start, buf
        i += 1


def _strip_comments(s: str) -> str:
    s = re.sub(r'/\*.*?\*/', ' ', s)
    s = s.split('//', 1)[0]
    s = s.split('/*', 1)[0]
    return s.strip()


_TOKEN = re.compile(r'\s*(defined|\d+[uUlL]*|[A-Za-z_]\w*|&&|\|\||==|!=|<=|>=|[!<>()])')


def _eval_if(expr: str, macros: dict[str, Macro]) -> bool:
    """Evaluate a #if expression. Raises ValueError on anything unexpected."""
    out = []
    pos = 0
    toks = []
    while pos < len(expr):
        m = _TOKEN.match(expr, pos)
        if not m:
            if expr[pos:].strip() == '':
                break
            raise ValueError(f'anlaşılmayan ifade: {expr!r}')
        toks.append(m.group(1))
        pos = m.end()
    i = 0
    while i < len(toks):
        t = toks[i]
        if t == 'defined':
            if i + 1 < len(toks) and toks[i + 1] == '(':
                name, i = toks[i + 2], i + 3          # skip ')'
                if i >= len(toks) or toks[i] != ')':
                    raise ValueError(f'defined( kapanmamış: {expr!r}')
            else:
                name, i = toks[i + 1], i + 1
            out.append('True' if name in macros else 'False')
        elif t == '&&':
            out.append(' and ')
        elif t == '||':
            out.append(' or ')
        elif t == '!':
            out.append(' not ')
        elif t in ('==', '!=', '<', '>', '<=', '>=', '(', ')'):
            out.append(t)
        elif t[0].isdigit():
            out.append(str(int(re.sub(r'[uUlL]+$', '', t), 0)))
        else:
            v = macros.get(t)
            try:
                out.append(str(int(v.value, 0)) if v and v.value else '0')
            except ValueError:
                out.append('0')
        i += 1
    code = ' '.join(out)
    if not re.fullmatch(r'[\sTrueFalsandotr0-9()=!<>]*', code):
        raise ValueError(f'güvensiz ifade: {code!r}')
    return bool(eval(code, {'__builtins__': {}}, {}))  # noqa: S307 — whitelisted above


_DIRECTIVE = re.compile(r'^\s*#\s*(\w+)\s*(.*)$')


def preprocess(text: str, predefined: dict[str, str]) -> Preprocessed:
    macros = {k: Macro(v, -1) for k, v in predefined.items()}
    warnings: list[str] = []
    # stack entries: [parent_active, this_branch_active, any_branch_taken]
    stack: list[list[bool]] = []

    def active() -> bool:
        return all(f[1] for f in stack)

    def cond(expr: str, lineno: int) -> bool:
        try:
            return _eval_if(expr, macros)
        except (ValueError, SyntaxError, IndexError) as e:
            warnings.append(f'satır {lineno + 1}: {e} — yanlış kabul edildi')
            return False

    for lineno, raw in _logical_lines(text):
        line = _strip_comments(raw)
        m = _DIRECTIVE.match(line)
        if not m:
            continue
        d, rest = m.group(1), m.group(2).strip()
        if d in ('if', 'ifdef', 'ifndef'):
            if not active():
                stack.append([False, False, True])
                continue
            if d == 'ifdef':
                c = rest.split()[0] in macros
            elif d == 'ifndef':
                c = rest.split()[0] not in macros
            else:
                c = cond(rest, lineno)
            stack.append([True, c, c])
        elif d == 'elif':
            f = stack[-1]
            if f[2] or not all(x[1] for x in stack[:-1]):
                f[1] = False
            else:
                f[1] = cond(rest, lineno)
                f[2] = f[1]
        elif d == 'else':
            f = stack[-1]
            f[1] = not f[2] and all(x[1] for x in stack[:-1])
            f[2] = True
        elif d == 'endif':
            stack.pop()
        elif not active():
            continue
        elif d == 'define':
            dm = re.match(r'([A-Za-z_]\w*)(\([^)]*\))?\s*(.*)$', rest)
            if dm:
                macros[dm.group(1)] = Macro(dm.group(3).strip(), lineno)
        elif d == 'undef':
            macros.pop(rest.split()[0], None)
    if stack:
        warnings.append(f'{len(stack)} adet #if kapanmamış')
    return Preprocessed(macros, warnings)


def env_defines(platformio_ini: str, env: str = ENV) -> dict[str, str] | None:
    """-D flags of [env:<env>], or None if the env does not exist."""
    m = re.search(r'^\[env:%s\]\s*$(.*?)(?=^\[|\Z)' % re.escape(env),
                  platformio_ini, re.MULTILINE | re.DOTALL)
    if not m:
        return None
    out = {'PLATFORMIO': '1'}
    for line in m.group(1).splitlines():
        line = line.split(';', 1)[0]
        for name, val in re.findall(r'-D\s*([A-Za-z_]\w*)(?:=(\S+))?', line):
            out[name] = val or '1'
    return out


# --- edits --------------------------------------------------------------------

def set_define(text: str, line: int, name: str, value: str) -> str:
    lines = text.split('\n')
    # pad a shorter value so the trailing comment keeps its column
    new, n = re.subn(r'^(\s*#\s*define\s+%s\s+)(\S+)' % re.escape(name),
                     lambda m: m.group(1) + value.ljust(len(m.group(2))),
                     lines[line], count=1)
    if n != 1:
        raise ValueError(f'satır {line + 1} beklenen #define {name} değil')
    lines[line] = new
    return '\n'.join(lines)


def comment_out(text: str, line: int, name: str) -> str:
    lines = text.split('\n')
    if not re.match(r'^\s*#\s*define\s+%s\b' % re.escape(name), lines[line]):
        raise ValueError(f'satır {line + 1} beklenen #define {name} değil')
    lines[line] = '// ' + lines[line] + '   // b1_flash: kapatıldı'
    return '\n'.join(lines)


# --- checks -------------------------------------------------------------------

@dataclass
class Finding:
    level: str                       # 'ok' | 'uyari' | 'hata'
    name: str
    message: str
    fix: Callable[[str], str] | None = None
    fix_label: str = ''


def _int(m: Macro | None) -> int | None:
    if m is None:
        return None
    try:
        return int(m.value.split()[0], 0)
    except (ValueError, IndexError):
        return None


def check(config_h: str, predefined: dict[str, str]) -> tuple[list[Finding], Preprocessed]:
    pp = preprocess(config_h, predefined)
    mac = pp.macros
    f: list[Finding] = []

    def must_equal(name: str, want: str, why: str):
        m = mac.get(name)
        if m is None:
            f.append(Finding('hata', name, f'tanımlı değil — {why}'))
        elif m.value.split()[0] == want:
            f.append(Finding('ok', name, f'{want} (satır {m.line + 1})'))
        else:
            f.append(Finding(
                'hata', name, f'{m.value.split()[0]} → {want} olmalı — {why}',
                fix=lambda t, m=m: set_define(t, m.line, name, want),
                fix_label=f'satır {m.line + 1}: {name} {want}'))

    def must_be_defined(name: str, why: str):
        m = mac.get(name)
        if m is None:
            f.append(Finding('hata', name, f'tanımlı değil — {why}'))
        else:
            f.append(Finding('ok', name, f'tanımlı (satır {m.line + 1})'))

    def must_be_off(name: str, why: str):
        m = mac.get(name)
        if m is None:
            f.append(Finding('ok', name, 'kapalı'))
        elif m.line < 0:
            f.append(Finding('hata', name, f'platformio.ini tanımlıyor — {why}'))
        else:
            f.append(Finding(
                'hata', name, f'AÇIK (satır {m.line + 1}) — {why}',
                fix=lambda t, m=m: comment_out(t, m.line, name),
                fix_label=f'satır {m.line + 1}: {name} yoruma alınır'))

    must_equal('CTRL_TYP_SEL', 'FOC_CTRL', 'SPD_MODE yalnız FOC ile çalışır')
    must_equal('CTRL_MOD_REQ', 'SPD_MODE',
               'köprü hız komutu gönderiyor (varsayılan VLT_MODE)')
    must_equal('BAT_CELLS', '10', '36 V 10S paket')
    must_be_defined('CONTROL_SERIAL_USART2', 'sol kablodan komut alınmalı')
    must_be_defined('FEEDBACK_SERIAL_USART2', 'hız feedback\'i (odometri) gelmeli')
    must_equal('USART2_BAUD', '115200', 'ESP32 HOVER_BAUD ile aynı olmalı')
    must_be_off('ENABLE_ODOMETRY',
                'feedback 22 bayta çıkar, ESP32 HoverFeedback (18 B) bozulur')
    must_be_off('TANK_STEERING', 'ESP32 steer+speed gönderiyor, tank değil')
    must_be_off('DUAL_INPUTS', 'tek giriş: USART2')

    # current limits vs the fuse
    i_mot, i_dc = _int(mac.get('I_MOT_MAX')), _int(mac.get('I_DC_MAX'))
    if i_mot is None or i_dc is None:
        f.append(Finding('hata', 'I_MOT_MAX/I_DC_MAX', 'okunamadı'))
    else:
        def fix_limits(t, a=mac['I_MOT_MAX'], b=mac['I_DC_MAX']):
            t = set_define(t, a.line, 'I_MOT_MAX', str(RECOMMENDED_I_MOT_MAX))
            return set_define(t, b.line, 'I_DC_MAX', str(RECOMMENDED_I_DC_MAX))
        label = (f'I_MOT_MAX {RECOMMENDED_I_MOT_MAX}, '
                 f'I_DC_MAX {RECOMMENDED_I_DC_MAX}')
        if 2 * i_dc > FUSE_A:
            f.append(Finding(
                'hata', 'I_DC_MAX',
                f'{i_mot}/{i_dc} A motor başına → iki motor {2 * i_dc} A, '
                f'{FUSE_A} A sigortayı aşar', fix=fix_limits, fix_label=label))
        elif i_mot > i_dc:
            f.append(Finding('hata', 'I_MOT_MAX', f'{i_mot} > I_DC_MAX {i_dc}',
                             fix=fix_limits, fix_label=label))
        elif i_dc != i_mot + 2:
            f.append(Finding(
                'uyari', 'I_DC_MAX',
                f'{i_mot}/{i_dc} A — EFeru kuralı I_DC_MAX = I_MOT_MAX + 2',
                fix=fix_limits, fix_label=label))
        else:
            f.append(Finding('ok', 'I_MOT_MAX/I_DC_MAX',
                             f'{i_mot}/{i_dc} A (toplam ≤ {2 * i_dc} A)'))

    st = mac.get('SERIAL_TIMEOUT')
    st_v = _int(st)
    if st_v is None:
        f.append(Finding('hata', 'SERIAL_TIMEOUT', 'tanımlı değil'))
    elif st_v > 160:
        f.append(Finding(
            'hata', 'SERIAL_TIMEOUT',
            f'{st_v} (≈{st_v * 5} ms) — bağlantı kopunca duruş gecikir, ≤160',
            fix=lambda t, m=st: set_define(t, m.line, 'SERIAL_TIMEOUT', '160'),
            fix_label=f'satır {st.line + 1}: SERIAL_TIMEOUT 160'))
    else:
        f.append(Finding('ok', 'SERIAL_TIMEOUT', f'{st_v} (≈{st_v * 5} ms)'))

    n_v = _int(mac.get('N_MOT_MAX'))
    if n_v is not None and n_v > 1000:
        f.append(Finding('uyari', 'N_MOT_MAX', f'{n_v} rpm — varsayılan 1000'))

    for w in pp.warnings:
        f.append(Finding('uyari', 'önişlemci', w))
    return f, pp


def apply_fixes(config_h: str, findings: list[Finding]) -> str:
    seen = set()
    for x in findings:
        if x.fix and x.fix_label not in seen:
            config_h = x.fix(config_h)
            seen.add(x.fix_label)
    return config_h
