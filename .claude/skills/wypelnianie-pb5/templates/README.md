# templates/ — szablon PB-5

Tu wgraj **oficjalny wypełnialny PDF PB-5** jako `PB-5.pdf`.

## Skąd wziąć

- e-Budownictwo: https://wnioski.gunb.gov.pl/report/wniosek/gunb_epb_5_oswiadczenie/
- GUNB — wzory: https://www.gunb.gov.pl/strona/wzory-wnioskow-zgloszen-i-zawiadomien
- BIP starostw / urzędów wojewódzkich (wersje PDF/DOC do pobrania)

> Pobranie automatyczne (curl/WebFetch) jest tu blokowane (allowlista sieci +
> ochrona przed botami → HTTP 403). Dlatego plik wgrywa człowiek.

## Dwie wersje pliku

- **PDF „płaski"** (do druku/ręcznego wypełnienia) — NIE ma pól AcroForm.
  `fill_pb5.py` go nie wypełni; trzeba najpierw dodać warstwę pól.
- **Wersja interaktywna** (z polami) — preferowana. Sprawdź:
  `python ../scripts/inspect_fields.py PB-5.pdf`

## field_map.json

Skopiuj `field_map.example.json` → `field_map.json` i podstaw realne nazwy pól
(z `inspect_fields.py`). Po lewej zostają nazwy logiczne — ich NIE zmieniaj.

Pliki `PB-5.pdf` i `field_map.json` nie są wersjonowane domyślnie — dodaj je do
repo świadomie, jeśli mają być współdzielone.
