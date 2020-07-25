from src.cell import cell
from time import sleep

if __name__ == "__main__":
    machine_cell = cell()
    machine_cell.cnc.superstate.enable()
    machine_cell.cmm.superstate.enable()
    machine_cell.buffer.superstate.enable()
    sleep(10)
    machine_cell.part_arrival()
