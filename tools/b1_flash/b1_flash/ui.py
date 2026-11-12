"""Terminal UI, and a scripted stand-in for tests.

The wizard only talks to the user through this interface, so tests drive the
whole flow by answering prompts by substring.
"""
from __future__ import annotations

import os
import sys
import unicodedata


def _norm(s: str) -> str:
    """Case/diacritic-insensitive: 'SİL', 'sil', 'SIL' all compare equal."""
    s = s.strip().replace('İ', 'I').replace('ı', 'i')
    s = unicodedata.normalize('NFKD', s)
    return ''.join(c for c in s if not unicodedata.combining(c)).casefold()


class ConsoleUI:
    def __init__(self, stream=sys.stdout, inp=input):
        self.out = stream
        self.inp = inp
        color = stream.isatty() and os.environ.get('NO_COLOR') is None
        self._c = (lambda code, s: f'\033[{code}m{s}\033[0m') if color else (lambda code, s: s)

    def _p(self, s=''):
        print(s, file=self.out, flush=True)

    def header(self, idx: int, total: int, title: str, ref: str = ''):
        self._p()
        self._p(self._c('1;36', f'━━━ Adım {idx}/{total}: {title} ' + '━' * 10))
        if ref:
            self._p(self._c('2', f'    ({ref})'))

    def say(self, s: str):
        self._p(s)

    def ok(self, s: str):
        self._p(self._c('32', '  ✓ ') + s)

    def warn(self, s: str):
        self._p(self._c('33', '  ⚠ ') + s)

    def err(self, s: str):
        self._p(self._c('1;31', '  ✗ ') + s)

    def cmd(self, s: str):
        self._p(self._c('2', '  $ ') + s)

    def confirm(self, q: str) -> bool:
        while True:
            a = _norm(self.inp(self._c('1', f'  ? {q} [e/h] ')))
            if a in ('e', 'evet', 'y', 'yes'):
                return True
            if a in ('h', 'hayir', 'n', 'no'):
                return False

    def ask(self, q: str) -> str:
        return self.inp(self._c('1', f'  ? {q} ')).strip()

    def ask_float(self, q: str) -> float:
        while True:
            a = self.ask(q).replace(',', '.')
            try:
                return float(a)
            except ValueError:
                self.warn('sayı gir (örn. 3.3)')

    def choose(self, q: str, options: dict[str, str]) -> str:
        for k, v in options.items():
            self._p(f'    [{k}] {v}')
        while True:
            a = _norm(self.ask(q))
            for k in options:
                if _norm(k) == a:
                    return k

    def typed_confirm(self, q: str, word: str) -> bool:
        a = self.ask(f'{q} — onaylamak için "{word}" yaz:')
        return _norm(a) == _norm(word)

    def pause(self, q: str = 'Hazır olunca Enter'):
        self.inp(self._c('1', f'  ↵ {q} '))


class ScriptedUI:
    """Answers prompts from (substring, answer) rules, first match wins.

    Defaults: confirm → True, typed_confirm → the word, choose → first option,
    pause → nothing. ask / ask_float without a rule is a test error.
    """

    def __init__(self, rules: list[tuple[str, object]] | None = None):
        self.rules = list(rules or [])
        self.log: list[str] = []
        self.asked: list[str] = []

    def _rule(self, q: str):
        self.asked.append(q)
        for sub, ans in self.rules:
            if sub in q:
                return True, ans
        return False, None

    def header(self, idx, total, title, ref=''):
        self.log.append(f'## {title}')

    def say(self, s):
        self.log.append(s)

    def ok(self, s):
        self.log.append('OK ' + s)

    def warn(self, s):
        self.log.append('WARN ' + s)

    def err(self, s):
        self.log.append('ERR ' + s)

    def cmd(self, s):
        self.log.append('$ ' + s)

    def confirm(self, q):
        hit, a = self._rule(q)
        return bool(a) if hit else True

    def ask(self, q):
        hit, a = self._rule(q)
        if not hit:
            raise AssertionError(f'cevapsız soru: {q}')
        return str(a)

    def ask_float(self, q):
        return float(self.ask(q))

    def choose(self, q, options):
        hit, a = self._rule(q)
        return a if hit else next(iter(options))

    def typed_confirm(self, q, word):
        hit, a = self._rule(q)
        return _norm(str(a)) == _norm(word) if hit else True

    def pause(self, q=''):
        self._rule(q)

    def text(self) -> str:
        return '\n'.join(self.log)
