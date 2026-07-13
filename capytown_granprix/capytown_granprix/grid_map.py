"""Seguimiento de celda y orientacion por conteo de movimientos.

La pista Gran Prix CapyTown es una rejilla de 6 columnas (A-F) por 4
filas (1-4), celdas de 60x60 cm (ver DETALLE_PISTA.md). No hay
localizacion absoluta (no hay marcas ArUco ni mapa cargado), asi que
la posicion se estima por conteo de celdas avanzadas y giros
ejecutados ("dead reckoning" logico), tal como sugiere la opcion
"Coordenadas por celdas" del documento de logica de pared derecha.

Convencion de coordenadas (igual a DETALLE_PISTA.md):
- Columnas A..F -> col 0..5, aumentan hacia el ESTE (derecha).
- Filas 1..4 -> row 0..3, aumentan hacia el SUR (abajo).
- NORTE disminuye la fila (sube en el plano), SUR la aumenta.
"""

from dataclasses import dataclass

HEADINGS = ['NORTE', 'ESTE', 'SUR', 'OESTE']

DELTA = {
    'NORTE': (0, -1),
    'ESTE': (1, 0),
    'SUR': (0, 1),
    'OESTE': (-1, 0),
}


def neighbor_cell(col: int, row: int, heading: str, num_columns: int = 6, num_rows: int = 4):
    """Celda vecina en ``heading`` desde (col, row), o None si cae fuera
    de la rejilla (esa direccion no tiene celda real, no una pared)."""
    dx, dy = DELTA[heading]
    new_col, new_row = col + dx, row + dy
    if 0 <= new_col < num_columns and 0 <= new_row < num_rows:
        return new_col, new_row
    return None


def turn_right(heading: str) -> str:
    return HEADINGS[(HEADINGS.index(heading) + 1) % 4]


def turn_left(heading: str) -> str:
    return HEADINGS[(HEADINGS.index(heading) - 1) % 4]


def turn_180(heading: str) -> str:
    return HEADINGS[(HEADINGS.index(heading) + 2) % 4]


def cell_name(col: int, row: int) -> str:
    letter = chr(ord('A') + col)
    return f'{letter}{row + 1}'


def cell_from_name(name: str) -> tuple:
    name = name.strip().upper()
    col = ord(name[0]) - ord('A')
    row = int(name[1:]) - 1
    return col, row


@dataclass
class GridTracker:
    """Estima celda actual y heading a partir de avances y giros."""

    col: int
    row: int
    heading: str
    num_columns: int = 6
    num_rows: int = 4

    @classmethod
    def from_cell_name(cls, name: str, heading: str, num_columns: int = 6, num_rows: int = 4):
        col, row = cell_from_name(name)
        return cls(col=col, row=row, heading=heading, num_columns=num_columns, num_rows=num_rows)

    @property
    def cell(self) -> str:
        return cell_name(self.col, self.row)

    def advance_cell(self) -> bool:
        """Actualiza la celda tras avanzar una celda (60 cm) al frente.

        Retorna True si el avance calculado caia fuera de la rejilla
        6x4 (se recorto al borde) -- senal de que la celda estimada ya
        probablemente no coincide con la posicion fisica real (deriva
        de odometria o un giro mal contado en algun punto anterior),
        util para diagnostico en el llamador.
        """
        dx, dy = DELTA[self.heading]
        new_col = self.col + dx
        new_row = self.row + dy
        fuera_de_rango = not (0 <= new_col < self.num_columns and 0 <= new_row < self.num_rows)
        self.col = max(0, min(self.num_columns - 1, new_col))
        self.row = max(0, min(self.num_rows - 1, new_row))
        return fuera_de_rango

    def apply_turn(self, direction: str) -> None:
        """direction in {'DERECHA', 'IZQUIERDA', 'ATRAS'} ('NINGUNO' no hace nada)."""
        if direction == 'DERECHA':
            self.heading = turn_right(self.heading)
        elif direction == 'IZQUIERDA':
            self.heading = turn_left(self.heading)
        elif direction == 'ATRAS':
            self.heading = turn_180(self.heading)
        elif direction == 'NINGUNO':
            return
        else:
            raise ValueError(f'direccion de giro desconocida: {direction}')
