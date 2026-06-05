# Wzory — biblioteka wzorów dokumentów (Warsztat Architekta)

Centralne miejsce na **oficjalne wzory** dokumentów używanych w procesie
budowlanym: oświadczeń, wniosków, wystąpień, wniosków o media itd.
Korzystają z nich skille z `.claude/skills/` (np. `wypelnianie-pb5`).

## Konwencja: grupowanie wg organu / instytucji

Każdy wzór trafia do podfolderu nazwanego od **organu lub instytucji**, do której
dokument jest kierowany (lub która go wydała):

```
Warsztat Architekta/Wzory/
├── GUNB/                     # formularze GUNB / e-Budownictwo (PB-1..PB-19, PB-5 itd.)
│   └── PB-5.pdf
├── Gestorzy mediów/          # wnioski o warunki/przyłącza: prąd, gaz, woda-kanalizacja, ciepło
├── Urząd Gminy/Miasta/       # wystąpienia, wnioski lokalne (np. zjazd, zajęcie pasa)
├── Konserwator zabytków/     # wystąpienia do WKZ
└── ...                       # kolejne instytucje wg potrzeb
```

Foldery zakładamy w miarę potrzeb — powyższe to punkt wyjścia, nie zamknięta lista.

## Co wkładamy do folderu wzoru

- **Oficjalny plik wzoru** — najlepiej **wypełnialny PDF (AcroForm)**.
  Nazwa = oznaczenie druku, np. `PB-5.pdf`. Jeśli druk ma kilka wersji, dopisz
  datę/rok, np. `PB-5_2021.pdf`.
- Opcjonalnie skan/PDF instrukcji urzędu, jeśli istnieje.

**Konfiguracja wypełniania** (np. `field_map.json` mapujące pola PDF) **nie**
trafia tutaj — należy do konkretnego skilla (to logika, nie wzór).

## Skąd brać wzory

- GUNB / e-Budownictwo: https://wnioski.gunb.gov.pl, https://www.gunb.gov.pl/strona/wzory-wnioskow-zgloszen-i-zawiadomien
- Gestorzy mediów / urzędy — ze stron danej instytucji.

> Pobieranie automatyczne bywa blokowane (allowlista sieci, ochrona przed botami).
> Wzory wgrywa człowiek.

## Powiązane skille

| Wzór | Folder | Skill |
|------|--------|-------|
| PB-5 — oświadczenie o prawie do dysponowania nieruchomością | `GUNB/PB-5.pdf` | `wypelnianie-pb5` |
