## Hlavní skript `main.m`

Tento skript slouží jako hlavní řídicí smyčka celého systému.

V tomto scriptu dochází ke zpracování překážek a plánování cesty robota bez jakéhokoliv výstupu, aby bylo dosaženo, co nejrychlejšího výpočtu.

## Naměřené časy

Data z kamery --- Vytvořená překážka ve voxel_grid  Δt = 0,002 ~ 0,003 s

Validace aktuální cesty                             Δt = 0,03 ~ 0,06 s  

Nalezení cesty                                      Δt = 0,05 ~ 0,5 s  

Pokud algoritmus nenajde cestu                      Δt = 1 ~ 1,2 s  

