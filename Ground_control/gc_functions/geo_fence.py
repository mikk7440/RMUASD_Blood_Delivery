from shapely.geometry import Point, Polygon  # Try: pip install Shapely or pip install Shapely --user
from matplotlib import pyplot as plt
from matplotlib.figure import Figure

# flight geometry
flight_geometry_coords = [(55.47180, 10.32514), (55.47157, 10.32445), (55.47134, 10.32488), (55.47115, 10.32430),
                          (55.47147, 10.32418), (55.47121, 10.32341), (55.47183, 10.32280), (55.47199, 10.32329), 
                          (55.47166, 10.32335), (55.47182, 10.32384), (55.47211, 10.32356), (55.47216, 10.32375), 
                          (55.47173, 10.32419), (55.47185, 10.32456), (55.47234, 10.32407), (55.47247, 10.32452),
                          (55.47206, 10.32466)]
geo_fence = Polygon(flight_geometry_coords)


def is_within_geo_fence(lat, lon):
    drone_position = Point(lat, lon)
    return drone_position.within(geo_fence)


def get_map_location(lat, lon):
    x, y = geo_fence.exterior.xy
    fig = Figure(figsize=(6, 4), dpi=100)
    fig.add_subplot(111).plot(x, y, 'b--', lat, lon, 'rx')
    return fig

def get_geo_fence_xy():
    return geo_fence.exterior.xy

# ax.set_title('Polygon')
# ax.plot(x, y, color='#6699cc', alpha=0.7,
# linewidth=3, solid_capstyle='round', zorder=2,)
# plt.show()
# Embed into tkinter https://matplotlib.org/3.1.0/gallery/user_interfaces/embedding_in_tk_sgskip.html
