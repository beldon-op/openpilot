"""Shared codepoint set for the high-resolution CJK display font.

Used by fonts/process.py (atlas generation at build time) and by the runtime
TTF fallback paths in system/ui and the carrot cluster/radar renderers, so the
generated atlas and the on-the-fly load never drift apart.

MiSans (the display font) covers GB2312 hanzi and kana but lacks Hangul and a
few UI symbols (⚙◀▶⌫⇧␣↳‑); Hangul is left to unifont and symbol-only strings
stay in Inter (see font_fallback in system/ui/lib/application.py).
"""
from pathlib import Path
import glob
import re

FONT_DIR = Path(__file__).resolve().parent
TRANSLATIONS_DIR = FONT_DIR.parents[1] / "ui" / "translations"

# Locales whose translation chars are baked into the CJK atlas.
CJK_TRANSLATION_LOCALES = ("zh-CHS", "zh-CHT", "ja")

_GB2312_LEVEL1: frozenset[str] | None = None


def gb2312_level1() -> frozenset[str]:
  """GB2312 level-1 hanzi: the 3,755 most common simplified characters."""
  global _GB2312_LEVEL1
  if _GB2312_LEVEL1 is None:
    chars = set()
    for row in range(0xB0, 0xD8):
      for col in range(0xA1, 0xFF):
        try:
          chars.add(bytes((row, col)).decode("gb2312"))
        except UnicodeDecodeError:
          pass
    _GB2312_LEVEL1 = frozenset(chars)
  return _GB2312_LEVEL1


def translation_cjk_chars() -> set[str]:
  """Distinct chars used by the zh/ja translations (kana included via ja)."""
  chars: set[str] = set()
  for locale in CJK_TRANSLATION_LOCALES:
    po_path = TRANSLATIONS_DIR / f"app_{locale}.po"
    try:
      chars.update(po_path.read_text(encoding="utf-8"))
    except FileNotFoundError:
      continue
  return chars


def cjk_font_codepoints() -> list[int]:
  """ASCII + UI symbols + GB2312-1 + zh/ja translation chars (no Hangul)."""
  chars = set(map(chr, range(32, 127)))
  chars.update("–‑✓×°§•X⚙✕◀▶✔⌫⇧␣○●↳çêüñ€£¥")
  # Extra symbols hard-coded in UI strings / seen in dynamic text (SSIDs).
  chars.update("±·—…’‘“”→←↑↓②③★☆㎡㎏℃℉")
  chars.update(gb2312_level1())
  chars.update(translation_cjk_chars())
  return sorted(ord(c) for c in chars)


def hangul_ui_chars() -> frozenset[str]:
  """Hangul syllables actually used by built-in UI strings.

  Scans the ko translation plus Hangul hard-coded in UI python sources, so the
  Korean fallback font atlas stays tiny (~hundreds of glyphs) instead of
  carrying all 11k syllables.
  """
  hangul = re.compile(r"[ᄀ-ᇿ㄰-㆏가-힣]")  # jamo + compatibility jamo + precomposed syllables
  chars: set[str] = set()
  po_path = TRANSLATIONS_DIR / "app_ko.po"
  try:
    chars.update(hangul.findall(po_path.read_text(encoding="utf-8")))
  except FileNotFoundError:
    pass
  ui_dir = FONT_DIR.parents[1] / "ui"  # openpilot/selfdrive/ui
  system_ui_dir = FONT_DIR.parents[2] / "system" / "ui"  # openpilot/system/ui
  for pattern in (str(ui_dir / "**" / "*.py"), str(system_ui_dir / "**" / "*.py")):
    for path in glob.glob(pattern, recursive=True):
      try:
        chars.update(hangul.findall(Path(path).read_text(encoding="utf-8")))
      except (OSError, UnicodeDecodeError):
        continue
  return frozenset(chars)
