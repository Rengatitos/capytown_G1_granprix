"""Mapa de conectividad del laberinto: se construye durante la
exploracion reactiva (Ronda 1) y se reutiliza en Ronda 2 para calcular
la ruta mas corta conocida con BFS, en vez de repetir la exploracion
reactiva de pared derecha (que no garantiza la ruta optima).

Solo tiene sentido si el layout de paredes NO cambia entre rondas --
ver README, seccion "Memoria de ruta (Ronda 2)". Si tu evaluacion si
reconfigura paredes entre rondas, usar ``usar_mapa_ronda2: false``.
"""

import json
import os
from collections import deque

from capytown_granprix.grid_map import DELTA, neighbor_cell, turn_left, turn_right


class MazeMap:
    """Grafo no dirigido de celdas conectadas (sin pared entre medio)."""

    def __init__(self, num_columns: int = 6, num_rows: int = 4):
        self._num_columns = num_columns
        self._num_rows = num_rows
        self._edges = set()  # frozenset({(col,row), (col,row)}) por par conectado

    def record(self, col: int, row: int, heading: str,
               derecha_libre: bool, frente_libre: bool, izquierda_libre: bool) -> None:
        """Registra, desde una interseccion ya confirmada por LiDAR, que
        direcciones absolutas quedan abiertas (sin pared) desde (col, row)."""
        opciones = (
            (derecha_libre, turn_right(heading)),
            (frente_libre, heading),
            (izquierda_libre, turn_left(heading)),
        )
        for libre, abs_heading in opciones:
            if not libre:
                continue
            vecino = neighbor_cell(col, row, abs_heading, self._num_columns, self._num_rows)
            if vecino is not None:
                self._edges.add(frozenset({(col, row), vecino}))

    def shortest_headings(self, start: tuple, goal: tuple):
        """BFS sobre lo mapeado hasta ahora. Retorna la lista de headings
        absolutos a tomar en cada tramo del camino mas corto conocido de
        ``start`` a ``goal``, o None si esa ruta no esta (aun) mapeada."""
        if start == goal:
            return []

        vecinos = {}
        for edge in self._edges:
            a, b = tuple(edge)
            vecinos.setdefault(a, []).append(b)
            vecinos.setdefault(b, []).append(a)

        visitados = {start}
        cola = deque([start])
        padre = {}
        while cola:
            actual = cola.popleft()
            if actual == goal:
                break
            for vecino in vecinos.get(actual, []):
                if vecino not in visitados:
                    visitados.add(vecino)
                    padre[vecino] = actual
                    cola.append(vecino)

        if goal not in visitados:
            return None

        camino = [goal]
        while camino[-1] != start:
            camino.append(padre[camino[-1]])
        camino.reverse()

        deltas_a_heading = {delta: heading for heading, delta in DELTA.items()}
        headings = []
        for actual, siguiente in zip(camino[:-1], camino[1:]):
            delta = (siguiente[0] - actual[0], siguiente[1] - actual[1])
            headings.append(deltas_a_heading[delta])
        return headings

    def save(self, path: str) -> None:
        path = os.path.expanduser(path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        data = [sorted(list(edge)) for edge in self._edges]
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f)

    @classmethod
    def load(cls, path: str, num_columns: int = 6, num_rows: int = 4) -> 'MazeMap':
        path = os.path.expanduser(path)
        mapa = cls(num_columns, num_rows)
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
        for (c1, r1), (c2, r2) in data:
            mapa._edges.add(frozenset({(c1, r1), (c2, r2)}))
        return mapa
