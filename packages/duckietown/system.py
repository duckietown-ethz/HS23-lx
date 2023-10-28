import traceback
from typing import Iterable

from .components import Component


class System:
    def __init__(self, components: Iterable[Component]):
        self._components: Iterable[Component] = components

    def _join(self, ignore_errors: bool = False):
        try:
            for component in self._components:
                component.join()
                print(f"[system]: Component {component.__class__.__name__}@{component.name.lower()} finished")
        except KeyboardInterrupt:
            raise
        except:
            if not ignore_errors:
                raise
            else:
                traceback.print_exc()

    def run(self):
        print("[system]: System booting...")
        try:
            for component in self._components:
                component.start()
                print(f"[system]: Component {component.__class__.__name__}@{component.name.lower()} started!")
            print("[system]: System running...")
            self._join()
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        for component in self._components:
            print(f"[system]: Stopping component {component.__class__.__name__}@{component.name.lower()}...")
            component.stop()
        try:
            self._join(ignore_errors=True)
        except KeyboardInterrupt:
            print("[system]: Already stopping...")
