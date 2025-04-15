InitializeVoxelGrid
Vytvoření prázdné 3D voxelové mřížky. Nulová matice – prostor bez překážek.
generate_voxel_mask
Vytvoření statických překážek pomocí kvádrů. Výstupem matice obsahující statické překážky.
get_next_waypoints
Načtení dvojice předem definovaných waypointů – start a cíl. Každý waypoint obsahuje kloubovou konfiguraci i souřadnici koncového efektoru [q1–q6, x, y, z].
add_start_and_goal
V matici označí start = 2 a cíl = 3. Pro následné jednodušší vizualizování.
rrt_6dof_connect_03
Využívá dva rostoucí stromy ze startu a z cíle, které se snaží spojit. Kolizní kontrola kloubů a objemového modelu robota ve 3D. Výstupem je:
-	q_path (joint-space cesta) (6xN)
-	joint_path (poloha kloubů v prostoru pro naplánovanou cestu) (Nx7x3)
forward_kinematics_ur5e
Výpočet dopředné kinematiky – výstupem poloha všech kloubů a koncového efektoru v prostoru
is_joint_in_collision
Prvotní kontrola, zda konkretní konfigurace je v překážce. Porovnání poloh kloubů v prostoru s voxel gridem
is_robot_model_in_collision
Druhá přesnější kontrola, zda konfigurace je v překážce. Model robota vytvořen válci mezi jednotlivými polohami kloubů a porovnán s voxel gridem.
add_cylinder_to_voxel_grid
Vytvoření válce mezi 2 body o konkrétním poloměru ve voxel gridu. 
fill_voxel_grid
vytvoření modelu člověka a doplnění do voxel gridu. Pomocí funkcí add_cylinder_to_voxel_grid a fill_sphere.
fill_sphere
Vytvoření sféry kolem bodu a doplnění do voxel gridu
is_path_valid
Kontrola, zda je naplánované cesta bez kolizí. Kontrola probíhá také dvou fázově.
plot_voxel_grid
vykreslení aktuálního stavu ve 3D prostředí včetně modelu robota a plánované cesty
