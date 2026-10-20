"""落地命令去重单测。"""

from px4_interface.command_dedupe import CommandDeduper


def test_land_fast_deduped_within_window():
    deduper = CommandDeduper(window_s=2.0)
    assert deduper.accept('land_fast', now=10.0)
    assert not deduper.accept('land_fast', now=11.0)
    assert deduper.accept('land_fast', now=12.5)


def test_other_commands_not_deduped():
    deduper = CommandDeduper(window_s=2.0)
    assert deduper.accept('arm', now=10.0)
    assert deduper.accept('arm', now=10.1)


def test_land_also_deduped():
    deduper = CommandDeduper(window_s=2.0)
    assert deduper.accept('land', now=10.0)
    assert not deduper.accept('land', now=10.5)
