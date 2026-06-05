#!/usr/bin/env python3
"""make_preview.py — generuj znormalizowany podgląd (ikonę) projektu architektonicznego.

Z rysunku elewacji frontowej (PDF z CAD lub obraz) albo z wizualizacji "od ulicy"
robi czysty plik JPG, który nadaje się na stronę tytułową projektu jako podgląd.

Dwa tryby:
  --mode elevation       rysunek techniczny (linie na białym tle): cała elewacja
                         widoczna, automatyczne przycięcie marginesów, dopasowanie
                         "contain" na białym tle.
  --mode visualization   render / zdjęcie "od ulicy": kadr wypełnia ramkę
                         ("cover" z przycięciem do zadanych proporcji).

Wejście może być PDF-em (renderowanym przez PyMuPDF) lub obrazem (PNG/JPG/TIFF).
Dla wielostronicowych/wieloelewacyjnych arkuszy użyj --page oraz --crop, aby
wybrać sam fragment z elewacją frontową.

Przykłady:
  # Elewacja frontowa z 2. strony PDF-a, domyślny rozmiar 1200x900:
  make_preview.py --mode elevation --input elewacje.pdf --page 2 -o previews/dom_kowalski.jpg

  # Wytnij górny-lewy kwadrant arkusza (frakcje 0..1: x,y,szer,wys) i przytnij białe marginesy:
  make_preview.py --mode elevation --input arkusz.pdf --crop 0,0,0.5,0.5 -o out.jpg

  # Wizualizacja od ulicy, kadr 16:9:
  make_preview.py --mode visualization --input render_ulica.jpg --size 1280x720 -o out.jpg
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys


# --- Bootstrap zależności (PyMuPDF + Pillow), w miarę bezboleśnie ---------------
def _ensure_deps():
    missing = []
    try:
        import fitz  # noqa: F401  (PyMuPDF)
    except Exception:
        missing.append("pymupdf")
    try:
        import PIL  # noqa: F401
    except Exception:
        missing.append("pillow")
    if not missing:
        return
    # Spróbuj cicho doinstalować w bieżącym środowisku.
    try:
        subprocess.run(
            [sys.executable, "-m", "pip", "install", "--quiet", *missing],
            check=True,
        )
    except Exception:
        sys.stderr.write(
            "\nBrakuje zależności: %s\n"
            "Zainstaluj je raz, np.:\n"
            "  python3 -m venv .venv && . .venv/bin/activate\n"
            "  pip install pymupdf pillow\n"
            "albo:  pip install %s\n\n" % (", ".join(missing), " ".join(missing))
        )
        sys.exit(2)


_ensure_deps()

import fitz  # noqa: E402
from PIL import Image, ImageChops, ImageColor, ImageDraw, ImageFont, ImageOps  # noqa: E402


# --- Parsowanie argumentów pomocniczych ----------------------------------------
def parse_size(spec: str) -> tuple[int, int]:
    s = spec.lower().replace(" ", "")
    for sep in ("x", "×", ","):
        if sep in s:
            w, h = s.split(sep, 1)
            return int(float(w)), int(float(h))
    raise argparse.ArgumentTypeError("Rozmiar podaj jako SZERxWYS, np. 1200x900")


def parse_crop(spec: str) -> tuple[float, float, float, float]:
    parts = [p for p in spec.replace(" ", "").split(",") if p != ""]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("Crop podaj jako x,y,szer,wys (frakcje 0..1)")
    x, y, w, h = (float(p) for p in parts)
    for v in (x, y, w, h):
        if not (0.0 <= v <= 1.0):
            raise argparse.ArgumentTypeError("Wartości crop muszą być w zakresie 0..1")
    return x, y, w, h


# --- Wczytanie źródła do obrazu PIL --------------------------------------------
def load_source(path: str, page: int, dpi: int, crop, bg) -> Image.Image:
    ext = os.path.splitext(path)[1].lower()
    if ext == ".pdf":
        return _render_pdf(path, page, dpi, crop)
    img = Image.open(path)
    img = ImageOps.exif_transpose(img)
    img = _flatten(img, bg)
    if crop:
        cx, cy, cw, ch = crop
        W, H = img.size
        box = (int(cx * W), int(cy * H), int((cx + cw) * W), int((cy + ch) * H))
        img = img.crop(box)
    return img


def _render_pdf(path: str, page: int, dpi: int, crop) -> Image.Image:
    doc = fitz.open(path)
    if page < 1 or page > doc.page_count:
        raise SystemExit(
            f"PDF ma {doc.page_count} stron(y); --page {page} poza zakresem."
        )
    pg = doc[page - 1]
    clip = None
    if crop:
        cx, cy, cw, ch = crop
        r = pg.rect
        clip = fitz.Rect(
            r.x0 + cx * r.width,
            r.y0 + cy * r.height,
            r.x0 + (cx + cw) * r.width,
            r.y0 + (cy + ch) * r.height,
        )
    zoom = dpi / 72.0
    pix = pg.get_pixmap(matrix=fitz.Matrix(zoom, zoom), clip=clip, alpha=False)
    return Image.frombytes("RGB", (pix.width, pix.height), pix.samples)


def _flatten(img: Image.Image, bg) -> Image.Image:
    if img.mode in ("RGBA", "LA") or (img.mode == "P" and "transparency" in img.info):
        base = Image.new("RGB", img.size, bg)
        rgba = img.convert("RGBA")
        base.paste(rgba, mask=rgba.split()[-1])
        return base
    return img.convert("RGB")


# --- Obróbka --------------------------------------------------------------------
def autotrim(img: Image.Image, bg, tol: int = 12) -> Image.Image:
    """Przytnij jednolite (zwykle białe) marginesy wokół rysunku."""
    bg_img = Image.new("RGB", img.size, bg)
    diff = ImageChops.difference(img, bg_img).convert("L")
    bbox = diff.point(lambda p: 255 if p > tol else 0).getbbox()
    return img.crop(bbox) if bbox else img


def fit_cover(img: Image.Image, size: tuple[int, int]) -> Image.Image:
    return ImageOps.fit(img, size, method=Image.LANCZOS, centering=(0.5, 0.5))


def fit_contain(img, size, bg, margin: float) -> Image.Image:
    W, H = size
    inner_w = max(1, int(W * (1 - 2 * margin)))
    inner_h = max(1, int(H * (1 - 2 * margin)))
    scaled = img.copy()
    scaled.thumbnail((inner_w, inner_h), Image.LANCZOS)
    canvas = Image.new("RGB", size, bg)
    canvas.paste(scaled, ((W - scaled.width) // 2, (H - scaled.height) // 2))
    return canvas


def add_border(img: Image.Image, width: int, color) -> Image.Image:
    return ImageOps.expand(img, border=width, fill=color)


def add_caption(img: Image.Image, text: str, bg, fg) -> Image.Image:
    if not text:
        return img
    W, H = img.size
    strip_h = max(28, int(H * 0.085))
    out = Image.new("RGB", (W, H + strip_h), bg)
    out.paste(img, (0, 0))
    draw = ImageDraw.Draw(out)
    font = _load_font(int(strip_h * 0.55))
    tb = draw.textbbox((0, 0), text, font=font)
    tw, th = tb[2] - tb[0], tb[3] - tb[1]
    draw.text(((W - tw) // 2, H + (strip_h - th) // 2 - tb[1]), text, font=font, fill=fg)
    return out


def _load_font(px: int):
    for name in (
        "DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/Library/Fonts/Arial.ttf",
        "Arial.ttf",
    ):
        try:
            return ImageFont.truetype(name, px)
        except Exception:
            continue
    return ImageFont.load_default()


# --- Main -----------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Generuj znormalizowany podgląd (ikonę) projektu z elewacji lub wizualizacji.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--input", "-i", required=True, help="Plik źródłowy: PDF lub obraz.")
    ap.add_argument("--output", "-o", required=True, help="Docelowy plik .jpg.")
    ap.add_argument(
        "--mode",
        choices=["elevation", "visualization", "auto"],
        default="auto",
        help="elevation = rysunek (contain+trim, białe tło); visualization = render (cover). "
        "auto = elevation dla PDF, visualization dla obrazów.",
    )
    ap.add_argument("--page", type=int, default=1, help="Strona PDF (od 1). Domyślnie 1.")
    ap.add_argument("--dpi", type=int, default=200, help="DPI renderu PDF. Domyślnie 200.")
    ap.add_argument("--size", type=parse_size, default=(1200, 900), help="Rozmiar wyjścia SZERxWYS. Domyślnie 1200x900.")
    ap.add_argument("--crop", type=parse_crop, default=None, help="Wycinek źródła: x,y,szer,wys we frakcjach 0..1.")
    ap.add_argument("--fit", choices=["cover", "contain"], default=None, help="Nadpisz dopasowanie wynikające z trybu.")
    ap.add_argument("--bg", default="#FFFFFF", help="Kolor tła (contain / spłaszczanie alfy). Domyślnie biały.")
    ap.add_argument("--margin", type=float, default=0.04, help="Margines wokół rysunku w trybie contain (frakcja). Domyślnie 0.04.")
    ap.add_argument("--no-trim", action="store_true", help="Nie przycinaj automatycznie marginesów (tryb elevation).")
    ap.add_argument("--quality", type=int, default=88, help="Jakość JPG 1..95. Domyślnie 88.")
    ap.add_argument("--border", type=int, default=0, help="Ramka w px (0 = brak).")
    ap.add_argument("--border-color", default="#DDDDDD", help="Kolor ramki.")
    ap.add_argument("--title", default=None, help="Opcjonalny podpis (np. nazwa projektu) w pasku na dole.")
    args = ap.parse_args(argv)

    mode = args.mode
    if mode == "auto":
        mode = "elevation" if args.input.lower().endswith(".pdf") else "visualization"
    fit = args.fit or ("contain" if mode == "elevation" else "cover")
    bg = ImageColor.getrgb(args.bg)

    img = load_source(args.input, args.page, args.dpi, args.crop, bg)

    if mode == "elevation" and not args.no_trim:
        img = autotrim(img, bg)

    if fit == "cover":
        img = fit_cover(img, args.size)
    else:
        img = fit_contain(img, args.size, bg, args.margin)

    if args.border > 0:
        img = add_border(img, args.border, ImageColor.getrgb(args.border_color))
    if args.title:
        img = add_caption(img, args.title, bg, ImageColor.getrgb("#333333"))

    out = args.output
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    img = img.convert("RGB")
    img.save(out, "JPEG", quality=max(1, min(95, args.quality)), optimize=True, progressive=True)
    print(f"OK: {out}  ({img.width}x{img.height}, tryb={mode}, fit={fit})")


if __name__ == "__main__":
    main()
