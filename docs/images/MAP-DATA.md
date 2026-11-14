# Map data in the city pictures

The pictures of Paris, Monaco, Marseille, Coimbra and Konya in this folder (`city-*.png`, `storm-marseille.png`, `sitl-marseille.png`, and the flights summarised in `weather-go-no-go.png`) were made from:

- **Buildings, water and land use**: Overture Maps Foundation, release 2026-10-21.0 (buildings and base themes). Contains information from [OpenStreetMap](https://www.openstreetmap.org/copyright) and [Overture Maps](https://overturemaps.org), which is made available here under the [Open Database License (ODbL)](https://opendatacommons.org/licenses/odbl/1-0/). © OpenStreetMap contributors, Overture Maps Foundation.
- **Terrain**: NASA Shuttle Radar Topography Mission (SRTM).
- **Weather**: NOAA GFS 0.25° forecasts and NOAA Integrated Surface Database (ISD) station records.

The pictures themselves are covered by this repository's license. The map data in them stays under the ODbL.

## How the map data was converted

The flights were planned and drawn on a converted copy of the Overture data: obstacle maps in this project's box format. This section describes that conversion, as section 4.6 of the ODbL asks. The converted maps are not distributed here; as a derivative of the ODbL data they remain under the ODbL.

1. **Area.** A square around each district centre, in metres east and north of it: Paris, Montparnasse (48.8430 N, 2.3255 E, 1,300 m across); Monaco, Monte-Carlo (43.7395 N, 7.4245 E, 1,200 m); Marseille, Vieux-Port (43.2925 N, 5.3735 E, 1,300 m); Coimbra, Alta and Baixa (40.2085 N, 8.4275 W, 1,200 m); Konya, Alaaddin and Mevlana (37.8718 N, 32.4985 E, 1,300 m).
2. **Grid.** The square is divided into 5 m cells.
3. **Terrain.** Each cell takes the highest SRTM elevation at its centre and four corners, measured from the lowest point of the square. Cells without a building are rounded up to the next 2 m.
4. **Building height.** Overture's `height` where it is given; otherwise `num_floors` × 3.2 m + 2.5 m; otherwise the larger of 10 m and the 75th percentile of the known heights in the square.
5. **Building cells.** Every cell a footprint touches gets the building's roof level: the highest terrain under the footprint plus the building's height. Roof levels are rounded up to the next 3 m, except for named buildings at least 30 m tall or 1,500 m² in footprint, which keep their exact roof level and their name (they are the rooftop dropoffs). Where footprints overlap, the higher roof wins, so nothing real is taller than its cell.
6. **Boxes.** Neighbouring cells with the same height, and for named buildings the same building, are merged into axis-aligned boxes.
7. **Water and open space.** Water polygons from the base theme mark water cells. Named land-use areas of the pedestrian, park, horticulture and recreation subtypes, at least 150 m², are the open-ground dropoffs.
