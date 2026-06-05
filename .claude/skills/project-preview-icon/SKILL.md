---
name: project-preview-icon
description: >-
  Tworzy znormalizowaną ikonę/podgląd projektu architektonicznego (JPG) na stronę
  tytułową — z rysunku elewacji frontowej (PDF z CAD) albo z wizualizacji "od ulicy".
  Use when the user wants a project cover/thumbnail/preview image for an architecture
  project title page. Wyzwalacze: "ikona do opisu", "podgląd projektu", "miniatura
  projektu", "elewacja frontowa do JPG", "wizualizacja od ulicy", "strona tytułowa",
  "okładka projektu", "rozpoznawanie projektów".
---

# Project preview icon (podgląd / ikona projektu)

Generuje czysty, znormalizowany obraz JPG, który trafia na **stronę tytułową
projektu (Word/.docx)** jako podgląd — pozwala na pierwszy rzut oka rozpoznać,
czego dotyczy projekt. Charakter estetyczno-porządkowy.

Są dwa źródła podglądu:

1. **Elewacja frontowa** — rysunek techniczny (zwykle PDF z eksportu CAD). Robimy
   z niego JPG: cała elewacja widoczna, przycięte białe marginesy, dopasowanie na
   białym tle (`--mode elevation`).
2. **Wizualizacja "od ulicy"** — render/foto pokazujący budynek od strony ulicy.
   Wybierasz właściwą wizualizację i robisz z niej kadr wypełniający ramkę
   (`--mode visualization`).

Narzędzia są w `scripts/`. Renderowanie PDF robi PyMuPDF (bez systemowego
popplera), obróbkę obrazu — Pillow. Skrypty doinstalują te paczki przy
pierwszym uruchomieniu; gdy środowisko jest "externally managed", utwórz venv:
`python3 -m venv .venv && . .venv/bin/activate && pip install pymupdf pillow`.

## Kiedy używać

Gdy użytkownik chce ikonę / okładkę / miniaturę / podgląd projektu na stronę
tytułową, albo prosi o zrobienie JPG-a z elewacji lub wybranie wizualizacji od
ulicy. Działa też dla pojedynczego pliku i dla całego folderu projektu.

## Workflow

### A. Z elewacji frontowej (PDF)

1. Ustal, która strona PDF-a i który fragment to elewacja **frontowa**. Jeśli nie
   wiesz, wyrenderuj podgląd całej strony i obejrzyj go (Read), żeby wybrać stronę
   i ewentualny wycinek:
   ```bash
   python scripts/make_preview.py -i elewacje.pdf --page 1 --mode elevation \
       -o /tmp/_podglad_str1.jpg
   ```
   Potem otwórz `/tmp/_podglad_str1.jpg` narzędziem Read, żeby ocenić zawartość.
2. Wygeneruj finalny podgląd. Jeśli na arkuszu jest kilka elewacji, wytnij samą
   frontową przez `--crop x,y,szer,wys` (frakcje 0..1 strony):
   ```bash
   python scripts/make_preview.py -i elewacje.pdf --page 2 --mode elevation \
       --crop 0,0,0.5,0.5 -o previews/<nazwa_projektu>.jpg
   ```

### B. Z wizualizacji "od ulicy" (Claude wybiera właściwą)

To Ty (Claude) oceniasz wzrokowo, która wizualizacja jest "od ulicy":

1. Znajdź kandydatów (np. `wizualizacje/*.jpg|*.png|*.pdf`). PDF-y najpierw
   wyrenderuj do podglądu jak wyżej.
2. **Obejrzyj** każdego kandydata narzędziem Read i wybierz ujęcie frontalne,
   pokazujące budynek od strony ulicy / wejścia (a nie z ogrodu, z lotu ptaka,
   detal czy wnętrze). W razie wątpliwości dopytaj użytkownika (AskUserQuestion).
3. Zrób z wybranej wizualizacji kadr:
   ```bash
   python scripts/make_preview.py -i wizualizacje/ulica_03.jpg --mode visualization \
       --size 1280x720 -o previews/<nazwa_projektu>.jpg
   ```

### C. Wstawienie na stronę tytułową (Word/.docx)

Domyślnie podgląd wstawiasz **ręcznie** w Wordzie (przeciągnij `previews/<projekt>.jpg`
na stronę tytułową). Jeśli użytkownik chce to zautomatyzować i ma szablon .docx,
użyj `scripts/insert_into_docx.py` — patrz sekcja "Automatyczne wstawianie".

## Konwencje

- Zapisuj podglądy do folderu `previews/` pod nazwą projektu, np.
  `previews/2025-118_dom_kowalski.jpg`, żeby łatwo je kojarzyć z projektem.
- Domyślny rozmiar 1200×900 (4:3). Dla wizualizacji panoramicznych użyj np.
  `--size 1600x900` (16:9).
- `--title "Nazwa projektu"` dokłada na dole dyskretny pasek z podpisem — bywa
  przydatny do szybkiego rozpoznawania w przeglądarce plików.

## Najważniejsze opcje make_preview.py

| Opcja | Znaczenie |
|-------|-----------|
| `--mode elevation\|visualization\|auto` | tryb (auto: PDF→elevation, obraz→visualization) |
| `--page N` | strona PDF (od 1) |
| `--crop x,y,szer,wys` | wycinek źródła we frakcjach 0..1 (wybór frontowej elewacji z arkusza) |
| `--size SZERxWYS` | rozmiar wyjścia (domyślnie 1200x900) |
| `--fit cover\|contain` | nadpisanie dopasowania wynikającego z trybu |
| `--dpi N` | DPI renderu PDF (domyślnie 200; zwiększ dla ostrości) |
| `--no-trim` | nie przycinaj automatycznie białych marginesów |
| `--title TEKST` | pasek z podpisem na dole |
| `--border N` / `--border-color` | ramka |
| `--quality 1..95` | jakość JPG (domyślnie 88) |

Pełna pomoc: `python scripts/make_preview.py -h`.

## Automatyczne wstawianie do .docx (opcjonalnie)

W szablonie strony tytułowej wstaw placeholder tekstowy `{{PODGLAD}}` tam, gdzie
ma trafić obraz, albo zostaw obrazek-zaślepkę. Potem:

```bash
# wariant z placeholderem:
python scripts/insert_into_docx.py --docx strona_tytulowa.docx \
    --image previews/<projekt>.jpg --placeholder "{{PODGLAD}}" --width-cm 12 \
    -o strona_gotowa.docx

# wariant: podmień pierwszy istniejący obraz w dokumencie:
python scripts/insert_into_docx.py --docx szablon.docx \
    --image previews/<projekt>.jpg --replace-first -o strona_gotowa.docx
```

Wymaga `python-docx` (skrypt doinstaluje sam).

## Wskazówki

- Elewacje CAD bywają cienkimi liniami na białym tle — `elevation` + autotrim daje
  schludny, wyśrodkowany kadr. Jeśli rysunek jest jasny/blady, podnieś `--dpi`.
- Jeśli przycięcie obetnie za dużo (np. są wymiary/opisy daleko od rysunku), dodaj
  `--no-trim` albo wskaż dokładny `--crop`.
- Dla spójnej "galerii" projektów trzymaj jeden rozmiar i jeden tryb dla danego
  typu źródła.
