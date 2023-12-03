import dataclasses
import json
from abc import ABC, abstractmethod


@dataclasses.dataclass
class GenericTelemetryAbs(ABC):

    @classmethod
    @abstractmethod
    def from_dict(cls, d: dict) -> 'GenericTelemetryAbs':
        pass

    def as_json(self) -> str:
        return json.dumps(dataclasses.asdict(self), sort_keys=True, indent=4)

    def __str__(self):
        return self.as_json()


@dataclasses.dataclass
class BatteryTelemetry(GenericTelemetryAbs):
    """
    Python class wrapping the JSON:

        {
            'cell_voltage': <float>, 
            'charging': <bool>, 
            'current': <float>, 
            'cycle_count': <int>, 
            'input_voltage': <float>, 
            'percentage': <int>, 
            'present': <bool>, 
            'temperature': <float>, 
            'time_to_empty': <int>, 
            'usb_out_1_voltage': <float>, 
            'usb_out_2_voltage': <float>
        }
    
    """
    cell_voltage: float
    charging: bool
    current: float
    cycle_count: int
    input_voltage: float
    percentage: int
    present: bool
    temperature: float
    time_to_empty: int
    usb_out_1_voltage: float
    usb_out_2_voltage: float

    @classmethod
    def from_dict(cls, d: dict) -> 'BatteryTelemetry':
        return BatteryTelemetry(**d)


@dataclasses.dataclass
class CPUTelemetry(GenericTelemetryAbs):
    """
    Python class wrapping the JSON:

        {
            'cores': <int>,
            'frequency': {
                'current': <int>,
                'min': <int>,
                'max': <int>
            },
            'percentage': <float>
        }

    """

    @dataclasses.dataclass
    class Frequency(GenericTelemetryAbs):
        """
        Python class wrapping the JSON:

            {
                'current': <int>,
                'min': <int>,
                'max': <int>
            }

        """
        current: int
        min: int
        max: int

        @classmethod
        def from_dict(cls, d: dict) -> 'CPUTelemetry.Frequency':
            return CPUTelemetry.Frequency(**d)

    cores: int
    frequency: Frequency
    percentage: float

    @classmethod
    def from_dict(cls, d: dict) -> 'CPUTelemetry':
        return CPUTelemetry(
            cores=d["cores"],
            frequency=CPUTelemetry.Frequency(**d["frequency"]),
            percentage=d["percentage"],
        )


@dataclasses.dataclass
class GPUTelemetry(GenericTelemetryAbs):
    """
    Python class wrapping the JSON:

        {
            'memory': {
                'free': <int>,
                'total': <int>,
                'used': <int>,
                'percentage': <float>
            },
            'percentage': <float>,
            'power': <float>,
            'temperature': <float>
        }
    
    """

    @dataclasses.dataclass
    class Memory(GenericTelemetryAbs):
        """
        Python class wrapping the JSON:

            {
                'free': <int>,
                'total': <int>,
                'used': <int>,
                'percentage': <float>
            }

        """
        free: int
        total: int
        used: int
        percentage: float

        @classmethod
        def from_dict(cls, d: dict) -> 'GPUTelemetry.Memory':
            return GPUTelemetry.Memory(**d)

    @classmethod
    def from_dict(cls, d: dict) -> 'GPUTelemetry':
        return GPUTelemetry(
            memory=GPUTelemetry.Memory(**d["memory"]),
            percentage=d["percentage"],
            power=d["power"],
            temperature=d["temperature"],
        )

    memory: Memory
    percentage: float
    power: float
    temperature: float


@dataclasses.dataclass
class MemoryTelemetry(GenericTelemetryAbs):
    """
    Python class wrapping the JSON:

        {
            'free': <int>,
            'total': <int>,
            'used': <int>,
            'percentage': <float>
        }

    """
    free: int
    total: int
    used: int
    percentage: float

    @classmethod
    def from_dict(cls, d: dict) -> 'MemoryTelemetry':
        return MemoryTelemetry(**d)


@dataclasses.dataclass
class DiskTelemetry(GenericTelemetryAbs):
    """
    Python class wrapping the JSON:

        {
            'free': <int>,
            'total': <int>,
            'used': <int>,
            'percentage': <float>
        }

    """
    free: int
    total: int
    used: int
    percentage: float

    @classmethod
    def from_dict(cls, d: dict) -> 'DiskTelemetry':
        return DiskTelemetry(**d)
