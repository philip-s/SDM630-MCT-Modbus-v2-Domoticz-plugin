# templates/ — konfiguracja wypełniania (field_map)

Ten folder trzyma **konfigurację skilla**, nie wzory dokumentów.

- `field_map.example.json` — wzór mapowania pól. Skopiuj go do `field_map.json`
  i podstaw realne nazwy pól z szablonu (z `inspect_fields.py`). Po lewej zostają
  nazwy logiczne — ich NIE zmieniaj. `checkbox.on` jest opcjonalne (skill i tak
  odczyta realny stan „on" z PDF).

## Gdzie jest sam wzór PB-5?

Oficjalny, wypełnialny wzór PDF żyje w centralnej bibliotece wzorów:

```
Warsztat Architekta/Wzory/GUNB/PB-5.pdf
```

Tam wgrywa go człowiek (pobranie z GUNB jest tu blokowane). Konwencja biblioteki:
`Warsztat Architekta/Wzory/README.md`.

## Czy szablon ma pola?

```bash
python ../scripts/inspect_fields.py "Warsztat Architekta/Wzory/GUNB/PB-5.pdf"
```
Jeśli „BRAK pól AcroForm" — szablon jest płaski; trzeba mu dodać warstwę pól
(patrz `../SKILL.md` → Krok 0).
