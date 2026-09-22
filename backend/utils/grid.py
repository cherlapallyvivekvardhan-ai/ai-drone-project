"""
Utility for converting geographic coordinates into a discretized 2D grid
(and back again) so that classic grid-based search algorithms like A* can
be run over real-world start/destination pairs.
"""

from dataclasses import dataclass
from models.schemas import Coordinate


@dataclass
class GridConverter:
    """
    Maps a bounding box between a start and destination coordinate onto a
    grid of `resolution` x `resolution` cells, with a small padding margin
    so the endpoints never sit exactly on the grid edge.
    """

    min_lat: float
    max_lat: float
    min_lon: float
    max_lon: float
    resolution: int = 25

    @classmethod
    def from_endpoints(cls, start: Coordinate, destination: Coordinate, resolution: int = 25, padding_ratio: float = 0.15) -> "GridConverter":
        lat_span = abs(destination.latitude - start.latitude) or 0.001
        lon_span = abs(destination.longitude - start.longitude) or 0.001

        lat_pad = lat_span * padding_ratio
        lon_pad = lon_span * padding_ratio

        min_lat = min(start.latitude, destination.latitude) - lat_pad
        max_lat = max(start.latitude, destination.latitude) + lat_pad
        min_lon = min(start.longitude, destination.longitude) - lon_pad
        max_lon = max(start.longitude, destination.longitude) + lon_pad

        return cls(min_lat, max_lat, min_lon, max_lon, resolution)

    def to_cell(self, coord: Coordinate) -> tuple[int, int]:
        """Converts a lat/lon coordinate into (row, col) grid indices."""
        lat_ratio = (coord.latitude - self.min_lat) / (self.max_lat - self.min_lat)
        lon_ratio = (coord.longitude - self.min_lon) / (self.max_lon - self.min_lon)

        row = int(round(lat_ratio * (self.resolution - 1)))
        col = int(round(lon_ratio * (self.resolution - 1)))

        row = max(0, min(self.resolution - 1, row))
        col = max(0, min(self.resolution - 1, col))
        return row, col

    def to_coordinate(self, cell: tuple[int, int]) -> Coordinate:
        """Converts (row, col) grid indices back into a lat/lon coordinate."""
        row, col = cell
        lat_ratio = row / (self.resolution - 1)
        lon_ratio = col / (self.resolution - 1)

        latitude = self.min_lat + lat_ratio * (self.max_lat - self.min_lat)
        longitude = self.min_lon + lon_ratio * (self.max_lon - self.min_lon)
        return Coordinate(latitude=latitude, longitude=longitude)
