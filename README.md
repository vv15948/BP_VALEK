## Projekt: joint_space_05
## Hlavní skript `main.m`

Tento skript slouží jako hlavní řídicí smyčka celého systému.

1. Inicializace voxelové mřížky a přidání statických překážek
2. Vnější smyčka - střídání waypointů - průchozích bodů, které se střídají po dojetí do cíle
3. Hlavní smyčka
     - načtení dat z kamery
     - vytvoření dynamické překážky a přidání do voxel gridu
     - validace předchozí cesty
         - validní - posun a vizualizace robota podél předchozí cesty
         - nevalidní
             - přepočet od aktuální konfigurace pomocí funkce ### 'rrt_6dof_connect_02'
             - vizualizace po nově přepočítané cestě
     - pokud jsme dojeli do cíle, tak opuštění smyčky
---
## Popis funkcí

### `initializeVoxelGrid`
Vytvoří prázdnou 3D voxelovou mřížku. Výstupem je nulová matice představující prostor bez překážek.

### `generate_voxel_mask`
Vytvoří statické překážky pomocí kvádrů. Výstupem je matice obsahující překážky.

### `get_next_waypoints`
Načte dvojici předem definovaných waypointů – start a cíl. Každý waypoint obsahuje kloubovou konfiguraci i souřadnici koncového efektoru `[q1–q6, x, y, z]`.

### `add_start_and_goal`
Označí ve voxel matici start (2) a cíl (3). Slouží pro jednodušší vizualizaci.

### `rrt_6dof_connect_03`
Implementuje RRT-Connect – využívá dva rostoucí stromy (ze startu a cíle), které se snaží propojit. Obsahuje kolizní kontrolu kloubů i objemového modelu robota ve 3D. Výstupem je:
- `q_path` – plánovaná trajektorie v joint-space (6xN)
- `joint_path` – pozice kloubů v prostoru (Nx7x3)

### `forward_kinematics_ur5e`
Výpočet dopředné kinematiky – výstupem jsou pozice všech kloubů a koncového efektoru v prostoru.

### `is_joint_in_collision`
Prvotní (rychlá) kontrola kolize – ověřuje, zda některý z kloubů není v překážce pomocí voxel gridu.

### `is_robot_model_in_collision`
Druhá (přesnější) kontrola – simuluje objem robota válci mezi klouby a ověřuje kolizi s voxel gridem.

### `add_cylinder_to_voxel_grid`
Přidá válec mezi dvěma body se zadaným poloměrem do voxel gridu.

### `fill_voxel_grid`
Vytvoří přibližný model člověka a doplní ho do voxel gridu pomocí `add_cylinder_to_voxel_grid` a `fill_sphere`.

### `fill_sphere`
Vytvoří kouli kolem zvoleného bodu a doplní ji do voxel gridu.

### `is_path_valid`
Ověří, zda je celá trajektorie bez kolizí. Kontrola probíhá ve dvou krocích (klouby + objem robota).

### `plot_voxel_grid`
Vykreslí aktuální stav prostředí ve 3D včetně modelu robota, překážek, startu/cíle a trajektorie.

