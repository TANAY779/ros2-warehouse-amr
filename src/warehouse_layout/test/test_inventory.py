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

"""The expected inventory: which tote, with which SKU, stands in which slot."""

from collections import Counter
import dataclasses

import pytest
from warehouse_description import robot_config
from warehouse_layout import inventory
from warehouse_layout import model
from warehouse_layout import rules
import yaml


@pytest.fixture(scope='module')
def layout():
    return model.load(robot=robot_config.load())


@pytest.fixture(scope='module')
def totes(layout):
    return inventory.expected_inventory(layout)


def test_documented_numbers(totes, layout):
    # 414 slots x fill ratio 0.5 = 207 totes, within the 230 tote tag IDs (D23).
    assert len(totes) == layout.inventory.tote_count == 207
    assert inventory.summary(layout, totes) == '207 totes in 414 slots, 60 SKUs'


def test_tote_ids_count_up_from_20_in_slot_order(totes):
    assert [tote.tote_id for tote in totes] == list(range(20, 20 + len(totes)))
    assert [tote.slot for tote in totes] == sorted(tote.slot for tote in totes)
    assert all(tote.tote_id in rules.TOTE_TAG_IDS for tote in totes)
    assert len({tote.slot for tote in totes}) == len(totes)


def test_every_sku_is_in_stock_and_evenly_spread(totes, layout):
    stock = Counter(tote.sku for tote in totes)
    assert sorted(stock) == list(inventory.sku_names(layout.inventory.skus))
    assert max(stock.values()) - min(stock.values()) <= 1      # 207 / 60: 3 or 4 each


def test_the_seed_decides_the_inventory(layout, totes):
    assert inventory.expected_inventory(layout) == totes
    other = inventory.expected_inventory(dataclasses.replace(layout, seed=layout.seed + 1))
    assert len(other) == len(totes) and other != totes


def test_too_many_totes_for_the_tag_ids(layout):
    too_many = dataclasses.replace(
        layout, inventory=dataclasses.replace(layout.inventory, tote_count=231))
    with pytest.raises(ValueError, match='231 totes, but only 230 tote tag IDs'):
        inventory.expected_inventory(too_many)


def test_file_lists_every_slot(totes, layout):
    data = yaml.safe_load(inventory.render(layout, totes))
    assert data['schema_version'] == inventory.SCHEMA_VERSION
    assert (data['world'], data['seed'], data['tote_count']) == (layout.name, layout.seed, 207)
    assert len(data['slots']) == data['slot_count'] == 414
    filled = {slot: entry for slot, entry in data['slots'].items() if entry is not None}
    assert filled == {tote.slot: {'tote': tote.tote_id, 'sku': tote.sku} for tote in totes}
    assert sum(data['skus'].values()) == 207
