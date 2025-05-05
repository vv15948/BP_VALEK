## Projekt: joint_space_08
## Hlavní změny

### `rrt_6dof_connect_02`
Rozšíření generování nových konfigurací do plného prostoru se 6 stupni volnosti.

### `is_robot_model_in_collision`
Funkce je stále využívána při generování nových bodů a taky při validaci předchozí cesty.
Zásadní změnou je zde nastavení offsetů pro klouby, aby kolizní model reprezentoval reálného robota, jak je vidět na obrázcích.

![model_robota_opraveny](https://github.com/user-attachments/assets/76cb5fb5-e324-421a-8e5d-bc8af66bae3a)

![model_robota_hrubsi_grid](https://github.com/user-attachments/assets/9f84fd6f-ab4f-48d0-88f9-e81dee217e2b)

### `fill_voxel_grid`
Omezení jednotlivých válců reprezentující lidksé tělo. Nemůže se stát, že by byly delší než je definováno ve funkci.
Důvod - občas se překážka vytvořila přes celý pracovní prostor, protože nebyla správně detekovaná. Nicměně v ten konkrétní okamžik ji to zpracovalo špatně. Tohle řešení by tomu mělo zamezit

### `get_next_waypoints`
Zde došlo k drobné změně. Nastavení waypointů podle reálných konfiguracích z laboratoře.

### `initializeVoxelGrid`
Doplnění jen okrajových bodů prostoru (=1) pro kontinuální plotování voxel gridu.


