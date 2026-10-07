"""The shared ``--runs`` parser (f1tenth_logger.test_campaign.run_select)."""

import pytest

from f1tenth_logger.test_campaign.run_select import (
    RunSelectionError, select_runs, short_id, tokenize)

AVAILABLE = [
    "P004-R001-20260922T143740",
    "P004-R003-20260923T145652",
    "P004-R004-20260923T165958",
    "P004-R067-20261001T111634",
    "P004-R089-20261001T111911",
    "P003-R009-20260928T170138",
]


def test_numbers_become_repetitions_in_the_order_given():
    assert select_runs("1-3-4-67-89", AVAILABLE) == [
        "P004-R001-20260922T143740",
        "P004-R003-20260923T145652",
        "P004-R004-20260923T165958",
        "P004-R067-20261001T111634",
        "P004-R089-20261001T111911",
    ]


def test_order_is_kept_and_duplicates_dropped():
    assert select_runs("89-1-89-R001", AVAILABLE) == [
        "P004-R089-20261001T111911", "P004-R001-20260922T143740"]


def test_full_and_short_ids_mix_with_numbers():
    spec = "4-P003-R009-20260928T170138-P004-R067,1"
    assert select_runs(spec, AVAILABLE) == [
        "P004-R004-20260923T165958",
        "P003-R009-20260928T170138",
        "P004-R067-20261001T111634",
        "P004-R001-20260922T143740",
    ]


def test_r_prefix_and_lower_case_are_accepted():
    assert select_runs("r3 R4", AVAILABLE) == [
        "P004-R003-20260923T145652", "P004-R004-20260923T165958"]


def test_a_bare_number_matches_that_repetition_of_every_prompt():
    available = ["P001-R002-20260921T145711", "P002-R002-20260922T142712"]
    assert select_runs("2", available) == available


def test_missing_runs_are_all_listed_with_what_is_available():
    with pytest.raises(RunSelectionError) as err:
        select_runs("1-5-P004-R099", AVAILABLE)
    text = str(err.value)
    assert "R005" in text and "P004-R099" in text
    assert "P004-R001-20260922T143740" in text  # the available list


def test_garbage_is_rejected_not_half_read():
    for spec in ("12x", "P003-R0091", "1-?-3", ""):
        with pytest.raises(RunSelectionError):
            select_runs(spec, AVAILABLE)


def test_tokenize_keeps_ids_with_hyphens_whole():
    assert tokenize("P003-R009-20260928T170138-7") == [
        ("full", "P003-R009-20260928T170138"), ("rep", "7")]


def test_short_id():
    assert short_id("P003-R009-20260928T170138") == "P003-R009"
    with pytest.raises(ValueError):
        short_id("P003-R009")
