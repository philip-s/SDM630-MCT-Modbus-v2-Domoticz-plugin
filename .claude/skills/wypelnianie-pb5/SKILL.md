---
name: wypelnianie-pb5
description: >-
  Wypełnia oświadczenie PB-5 o posiadanym prawie do dysponowania nieruchomością
  na cele budowlane (GUNB, Dz.U. 2021 poz. 1170) na wypełnialnym PDF (AcroForm),
  na podstawie danych z pliku Danet.txt. Generuje osobny PDF dla każdego
  sygnatariusza, gotowy do wysłania inwestorowi/inwestorom do podpisu. Użyj, gdy
  pojawia się prośba o przygotowanie/wypełnienie oświadczenia PB-5, B-3 lub
  „oświadczenia o prawie do dysponowania nieruchomością".
---

# Wypełnianie oświadczenia PB-5

Skill przygotowuje **oświadczenie o posiadanym prawie do dysponowania
nieruchomością na cele budowlane (PB-5)** — wypełnia oficjalny, wypełnialny PDF
(AcroForm) danymi z `Danet.txt` i zapisuje po jednym dokumencie na sygnatariusza.
Dokument trafia następnie do inwestora/inwestorów **do podpisu** (skill NIE
podpisuje).

Szczegóły formularza, podstawa prawna i struktura pól: **`references/PB-5.md`**.

## Zasady (przeczytaj zanim zaczniesz)

- **Aktualny wzór to PB-5** (Dz.U. 2021 poz. 1170, od 1.07.2021). `PB-3` to wniosek
  o rozbiórkę — to NIE to. Dawny odpowiednik PB-5 nazywał się „B-3".
- **Jeden podpis = jeden egzemplarz.** Gdy prawo przysługuje kilku osobom/
  współwłaścicielom i każda ma złożyć oświadczenie, każdy sygnatariusz dostaje
  osobny PDF. Skill robi to automatycznie z sekcji `[sygnatariusze]`.
- **Podpisuje inwestor, nie skill.** Forma elektroniczna wymaga podpisu
  kwalifikowanego / zaufanego / osobistego. Skill tylko wypełnia.
- **Nie zmyślaj danych.** Brakujące pola zostają puste, a skill zgłasza
  ostrzeżenie. Nie wpisuj numerów działek, organu ani tytułu prawnego „z głowy".

## Potrzebne pliki wejściowe

1. **`Danet.txt`** — dane sprawy. Format i przykład: `examples/Danet.example.txt`.
   Jeśli realny `Danet.txt` projektu ma inny układ, dostosuj parser
   (`scripts/parse_danet.py`) — jest celowo tolerancyjny (`klucz = wartość`,
   sekcje `[...]`, powtarzalne `dzialka`/`osoba`).
2. **`templates/PB-5.pdf`** — oficjalny wypełnialny wzór PB-5. **Wgrywa go człowiek**
   (pobranie z GUNB/e-Budownictwo jest blokowane w tym środowisku). Patrz
   `templates/README.md`.
3. **`templates/field_map.json`** — mapowanie pól (utworzone raz dla danego
   szablonu; wzór: `templates/field_map.example.json`).

## Procedura

### Krok 0 — Przygotowanie szablonu (jednorazowo dla danego PDF)

1. Sprawdź, czy szablon ma pola AcroForm:
   ```bash
   python scripts/inspect_fields.py templates/PB-5.pdf
   ```
   - Jeśli wypisze listę pól (z `stany_on` przy checkboxach) — szablon jest
     wypełnialny, przejdź dalej.
   - Jeśli wypisze „BRAK pól AcroForm" — szablon jest **płaski**. Trzeba mu
     jednorazowo dodać warstwę pól (np. w LibreOffice Draw / Acrobat:
     rozmieść pola tekstowe i checkboxy, nazwij je, zapisz jako PDF z
     formularzem) albo poproś użytkownika o wersję wypełnialną. Bez pól
     `fill_pb5.py` nie zadziała.

2. Utwórz `templates/field_map.json` na bazie `field_map.example.json`:
   po lewej zostają nazwy logiczne (NIE zmieniaj), po prawej wstaw realne
   nazwy pól z `inspect_fields.py`. Pola, których w szablonie nie ma, zostaw —
   zostaną pominięte z ostrzeżeniem. `checkbox.on` jest opcjonalne: skill i tak
   odczyta realny stan „on" z PDF.

### Krok 1 — Sprawdź dane

```bash
python scripts/parse_danet.py Danet.txt
```
Wypisuje znormalizowane dane (JSON) i `OSTRZEŻENIE:` o brakach. Zweryfikuj
z użytkownikiem organ, działki, tytuł prawny i listę sygnatariuszy, jeśli czegoś
brakuje.

### Krok 2 — Wypełnij

```bash
python scripts/fill_pb5.py \
  --danet Danet.txt \
  --template templates/PB-5.pdf \
  --field-map templates/field_map.json \
  --out out/
```
Powstaje `out/PB-5_<imie_nazwisko>.pdf` dla każdego sygnatariusza.

### Krok 3 — Kontrola i wysyłka

- Otwórz wynikowe PDF-y i sprawdź wzrokowo (polskie znaki w polach tekstowych
  renderują się poprawnie w czytnikach honorujących `NeedAppearances` — Adobe
  Reader, przeglądarki; oficjalny szablon GUNB ma font z polskimi znakami).
- Zaadresuj do właściwego inwestora/sygnatariusza i wyślij **do podpisu**.
  Przy współwłasności dopilnuj, by każdy współwłaściciel-sygnatariusz otrzymał
  swój egzemplarz.

## Pliki skilla

- `scripts/parse_danet.py` — parser `Danet.txt` → model danych (+ walidacja).
- `scripts/fill_pb5.py` — wypełnia PDF, jeden plik na sygnatariusza.
- `scripts/inspect_fields.py` — lista pól AcroForm szablonu (do field_map.json).
- `scripts/pdf_utils.py` — wspólne funkcje PDF (stany checkboxów).
- `templates/field_map.example.json` — wzór mapowania pól.
- `examples/Danet.example.txt` — wzór pliku danych.
- `references/PB-5.md` — opis formularza, podstawa prawna, źródła.

## Zależności

- Python 3 + `pypdf`. Instalacja: `pip install pypdf`
  (jeśli pojawi się błąd `_cffi_backend`: `pip install --force-reinstall cffi`).

## Ograniczenia i uwagi

- Tabela działek w szablonie ma stałą liczbę wierszy (`table.max_rows` w
  field_map). Nadmiar skill zgłasza ostrzeżeniem — dołącz jako osobny załącznik.
- Skill nie składa podpisu ani nie wysyła pism do urzędu — przygotowuje dokument
  do podpisu przez inwestora.
- Gdy różni sygnatariusze mają różne adresy/dane, rozszerz `Danet.txt`
  (osobne dane per osoba) lub użyj osobnych plików `Danet.txt`.
