# PB-5 — Oświadczenie o posiadanym prawie do dysponowania nieruchomością na cele budowlane

## Status prawny

- **Aktualny wzór:** PB-5.
- **Podstawa prawna:** Rozporządzenie Ministra Rozwoju, Pracy i Technologii z dnia 25 czerwca 2021 r.
  w sprawie wzoru oświadczenia o posiadanym prawie do dysponowania nieruchomością na cele budowlane
  (**Dz.U. 2021 poz. 1170**), obowiązuje od **1 lipca 2021 r.** Wzór nadal aktualny (stan: 2026 r.).
- **Poprzednik:** dawny druk „B-3".
- **Uwaga na numerację:** `PB-3` to *wniosek o pozwolenie na rozbiórkę* — NIE oświadczenie.
  Oświadczenie o prawie do nieruchomości to **PB-5**.
- **Materialna podstawa:** art. 32 ust. 4 pkt 2 oraz definicja w art. 3 pkt 11 ustawy
  Prawo budowlane (prawo do dysponowania nieruchomością na cele budowlane).

## Skąd wziąć oficjalny wzór

- Generator/portal: e-Budownictwo (`wnioski.gunb.gov.pl`, `e-budownictwo.gunb.gov.pl`).
- BIP urzędów (starostwa, urzędy wojewódzkie) — wersja PDF/DOC do pobrania.
- ISAP — załącznik do Dz.U. 2021 poz. 1170.

> Te domeny blokują automatyczne pobieranie (allowlista sieci + ochrona przed botami → HTTP 403),
> dlatego oficjalny plik bazowy dostarcza człowiek i wgrywa do `templates/`.

### Dwie wersje pliku
- **PDF „płaski"** (do druku / ręcznego wypełnienia) — NIE ma pól AcroForm.
- **Wersja interaktywna** — dostępna tylko wewnątrz portalu e-Budownictwo, trudna do wyeksportowania.

Ten skill operuje na **wypełnialnym PDF (AcroForm)**. Jeśli wgrany plik jest płaski,
trzeba mu jednorazowo dodać warstwę pól (patrz SKILL.md → „Przygotowanie szablonu").

## Struktura formularza (sekcje)

1. **Organ administracji architektoniczno-budowlanej** — do kogo kierowane oświadczenie
   (np. „Starosta …", „Prezydent Miasta …", „Wojewoda …").
   - Numer rejestru organu — pole wypełniane przez organ (zostawiamy puste).

2. **Dane podmiotu składającego oświadczenie (inwestor)**
   - Rodzaj podmiotu: ☐ osoba fizyczna ☐ osoba prawna ☐ jednostka organizacyjna nieposiadająca osobowości prawnej
   - Imię i nazwisko (osoba fizyczna) **albo** nazwa (osoba prawna / jednostka)
   - Adres: kraj, województwo, powiat, gmina, miejscowość, ulica, nr domu, nr lokalu, kod pocztowy, poczta
   - (gdy osoba prawna/jednostka) **Dane osoby upoważnionej do reprezentowania** — imię i nazwisko

3. **Oznaczenie nieruchomości** — tabela (wiele wierszy, np. inwestycje liniowe):
   | jednostka ewidencyjna | obręb ewidencyjny | nr działki ewidencyjnej |

4. **Tytuł prawny do dysponowania** — zaznaczyć JEDEN:
   1. ☐ własności
   2. ☐ współwłasności *(wskazać współwłaścicieli: imię i nazwisko / nazwa oraz adres; wymagana zgoda wszystkich)*
   3. ☐ użytkowania wieczystego
   4. ☐ trwałego zarządu
   5. ☐ ograniczonego prawa rzeczowego
   6. ☐ stosunku zobowiązaniowego przewidującego uprawnienie do wykonywania robót budowlanych

5. **Przy współwłasności** — oświadczenie o posiadaniu zgody wszystkich pozostałych współwłaścicieli
   oraz ich wskazanie (imię i nazwisko / nazwa + adres).

6. **Pouczenie + podpis**
   - Klauzula: „Świadomy(-a) odpowiedzialności karnej za podanie w niniejszym oświadczeniu nieprawdy,
     zgodnie z art. 233 Kodeksu karnego, potwierdzam własnoręcznym podpisem prawdziwość danych
     zamieszczonych powyżej."
   - Miejscowość i data, **podpis** składającego.

## Zasady wypełniania (ważne dla automatyzacji)

- **Jeden sygnatariusz = jeden egzemplarz.** Gdy prawo przysługuje kilku osobom/współwłaścicielom
  i każdy ma złożyć oświadczenie, **każdy składa je na osobnym formularzu** → skill generuje
  po jednym PDF na sygnatariusza.
- **Podpis składa inwestor, nie skill.** Skill wyłącznie *przygotowuje i wypełnia* dokument.
  Forma elektroniczna wymaga podpisu kwalifikowanego / zaufanego / osobistego.
- Każdą działkę wpisujemy osobnym wierszem (nr działki, obręb, jednostka ewidencyjna).
- Warto mieć nr KW — przyspiesza weryfikację stanu prawnego przez organ (pole pomocnicze, nie zawsze
  w samym formularzu).

## Źródła

- Dz.U. 2021 poz. 1170 — ISAP: https://isap.sejm.gov.pl/isap.nsf/DocDetails.xsp?id=WDU20210001170
- e-Budownictwo / PB-5: https://wnioski.gunb.gov.pl/report/wniosek/gunb_epb_5_oswiadczenie/
- GUNB — wzory: https://www.gunb.gov.pl/strona/wzory-wnioskow-zgloszen-i-zawiadomien
- gov.pl (PB-5): https://www.gov.pl/web/uw-zachodniopomorski/oswiadczenie-o-posiadanym-prawie-do-dysponowania-nieruchomoscia-na-cele-budowlane-pb-5
