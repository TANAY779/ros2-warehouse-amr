# Copyright 2026 Tanay Baisware
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The expected inventory: which tote, holding which SKU, stands in each slot at the start.

It is data for Phase 8, whose inventory manager compares what the robots see with it. The
world shows these totes only when ``style.show_totes`` is true (bare shelves until Phase 7).

The inventory is random but repeatable: the same layout.yaml gives the same file on every
machine and in every run.

- Exactly ``inventory.tote_count`` slots hold a tote (slots x fill_ratio, rounded: 207 of the
  414), chosen at random from all slots.
- The SKUs are dealt over the totes like cards, in a random order. Every SKU is then in stock
  (when there are at least as many totes as SKUs), and no SKU has more than one tote more
  than another.
- Totes are numbered in slot-ID order from 20. A tote's ID is also the ID of the ArUco tag on
  it: plan D23 reserves IDs 20-249 for totes, which is why the ``totes`` fit rule allows at
  most 230 totes.

Only ``random.Random(seed).random()`` is used. Python promises that ``random()`` gives the
same sequence for the same seed in every version; ``shuffle()``, ``sample()`` and
``choice()`` carry no such promise. So a shuffle here is a sort on random numbers.
"""

from collections import Counter
from dataclasses import dataclass
import random

from warehouse_layout import formatting
from warehouse_layout.rules import TOTE_TAG_IDS

SCHEMA_VERSION = 1


@dataclass(frozen=True)
class PlacedTote:
    """One tote on the shelves: the slot it stands in, its ID (= its tag's ID) and its SKU."""

    slot: str
    tote_id: int
    sku: str


def sku_names(count):
    """Return the SKU names SKU-001, SKU-002, ... for ``count`` SKUs."""
    width = max(3, len(str(count)))
    return tuple(f'SKU-{number:0{width}d}' for number in range(1, count + 1))


def slot_order(slot):
    """Return a slot's sort key: row, bay, level, position (the order of the slot IDs)."""
    return (slot.row, slot.bay, slot.level, slot.position)


def expected_inventory(layout):
    """Return the totes on the shelves as a tuple of PlacedTote, in slot-ID order."""
    slots = sorted(layout.slots, key=slot_order)
    count = layout.inventory.tote_count
    if count > len(TOTE_TAG_IDS):
        # The totes fit rule reports this first; the check keeps the IDs valid on their own.
        raise ValueError(f'{count} totes, but only {len(TOTE_TAG_IDS)} tote tag IDs')
    rng = random.Random(layout.seed)

    # Which slots hold a tote: a random number for every slot; the lowest `count` win.
    draws = [rng.random() for _ in slots]
    chosen = sorted(sorted(range(len(slots)), key=lambda i: (draws[i], i))[:count])

    # Which SKU each tote holds: deal the SKUs round-robin over the totes in a random order.
    skus = sku_names(layout.inventory.skus)
    draws = [rng.random() for _ in chosen]
    deal_order = sorted(range(len(chosen)), key=lambda i: (draws[i], i))
    sku_of = {tote: skus[rank % len(skus)] for rank, tote in enumerate(deal_order)}

    return tuple(PlacedTote(slots[slot].name, TOTE_TAG_IDS[tote], sku_of[tote])
                 for tote, slot in enumerate(chosen))


def summary(layout, totes):
    """Return a one-line summary of the inventory, for the generate command."""
    return (f'{len(totes)} totes in {len(layout.slots)} slots, '
            f'{layout.inventory.skus} SKUs')


def render(layout, totes):
    """Return expected_inventory.yaml's text for the totes of expected_inventory()."""
    in_slot = {tote.slot: tote for tote in totes}
    stock = Counter(tote.sku for tote in totes)
    lines = formatting.yaml_comment(formatting.notice() + [
        '',
        'The expected inventory: which tote, holding which SKU, stands in each slot when the',
        'simulation starts. Data for Phase 8; the world shows the totes only when',
        'style.show_totes is true in layout.yaml (bare shelves until Phase 7).',
        "Random but repeatable: made from layout.yaml's seed, inventory.fill_ratio and",
        "inventory.skus. A tote's ID is also the ID of the ArUco tag on it (plan D23: 20-249).",
    ])
    lines += [
        '',
        f'schema_version: {SCHEMA_VERSION}',
        f'world: {formatting.yaml_str(layout.name)}',
        f'seed: {layout.seed}',
        f'fill_ratio: {formatting.yaml_float(layout.inventory.fill_ratio)}',
        f'slot_count: {len(layout.slots)}',
        f'tote_count: {len(totes)}',
        '',
        '# Every SKU, and how many totes hold it.',
        'skus:',
    ]
    lines += [f'  {sku}: {stock[sku]}' for sku in sku_names(layout.inventory.skus)]
    lines += [
        '',
        '# Every slot by slot ID: the tote that stands in it and the SKU that tote holds, or',
        '# null for an empty slot. Slot ID <row><bay>-<level>-<position>: C07-2-1 is row C,',
        '# bay 07, level 2 (the first board), position 1 (the south one).',
        'slots:',
    ]
    for slot in sorted(layout.slots, key=slot_order):
        tote = in_slot.get(slot.name)
        value = 'null' if tote is None else formatting.yaml_flow(
            [('tote', str(tote.tote_id)), ('sku', formatting.yaml_str(tote.sku))])
        lines.append(f'  {formatting.yaml_str(slot.name)}: {value}')
    return '\n'.join(lines) + '\n'
