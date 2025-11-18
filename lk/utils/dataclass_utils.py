from dataclasses import dataclass, asdict, is_dataclass, make_dataclass
from typing import Any


def combine_dataclass_instances(*instances: object) -> object:
    """
        Generic helper to combine multiple dataclass instances with validation.

        Methods are copied from the first instance to the extended instance.

    """

    if len(instances) < 2:
        raise ValueError("Must provide at least two dataclass instances to combine")
    
    for instance in instances:
        if not is_dataclass(instance):
            raise TypeError("All provided instances must be dataclass instances")

    # Get the first instance's type to preserve its class and helper methods
    first_instance_type = type(instances[0])
    
    combined_fields: dict[str, Any] = {}
    dataclass_fields: list[tuple[str, type]] = []
    
    # Collect all fields from all instances
    # Use __dict__ instead of __dataclass_fields__ to capture dynamically added fields
    for instance in instances:
        for field_name, item in instance.__dict__.items():
            # Skip private attributes and non-JointDescription/LinkDescription items
            if field_name.startswith('_'):
                continue
            
            # Check for field name conflicts
            if field_name in combined_fields:
                raise ValueError(
                    f"Duplicate field name '{field_name}' found when "
                    f"combining instances. All field names must be unique."
                )

            combined_fields[field_name] = item
            field_type = type(item) if item is not None else Any
            dataclass_fields.append((field_name, field_type))

    ExtendedDataclass = make_dataclass(first_instance_type.__name__+"Extended", dataclass_fields)  # type: ignore[arg-type]
    
    # Copy over all callable attributes from the original class and its base classes
    # Traverse MRO to get methods from the entire inheritance hierarchy
    # Skip only dataclass-generated attributes that would conflict
    dataclass_internals = {'__init__', '__dataclass_fields__', '__dataclass_params__', '__post_init__', '__repr__', '__eq__', '__hash__'}
    copied_attrs = set()
    
    # Iterate through MRO (most specific to least specific) to preserve method resolution order
    for cls in first_instance_type.__mro__:
        # Skip object class
        if cls is object:
            continue
            
        for attr_name, attr_value in cls.__dict__.items():
            # Skip dataclass-generated attributes that would conflict
            if attr_name in dataclass_internals:
                continue
            
            # Skip if we've already copied this attribute (prefer more specific definitions)
            if attr_name in copied_attrs:
                continue
            
            # Copy all callable attributes (methods, properties, classmethods, staticmethods, etc.)
            if callable(attr_value) or isinstance(attr_value, (property, classmethod, staticmethod)):
                setattr(ExtendedDataclass, attr_name, attr_value)
                copied_attrs.add(attr_name)
    
    return ExtendedDataclass(**combined_fields)


if __name__ == "__main__":

    @dataclass
    class cats:
        cat1: int = None
        cat2: int = None
        cat3: int = None

        def meow(self):
            print("meow")

    @dataclass
    class dogs:
        dog1: int = None
        dog2: int = None
        dog3: int = None

        def bark(self):
            print("bark")

    cat1 = cats(cat1=1, cat2=2, cat3=3)
    cat2 = cats(cat1=4, cat2=5, cat3=6)
    cat3 = cats(cat1=7, cat2=8, cat3=9)

    try:
        combined = combine_dataclass_instances(cat1, cat2, cat3)
    except Exception as e:
        print(e)


    cat1 = cats(cat1=1, cat2=2, cat3=3)
    dog1 = dogs(dog1=4, dog2=5, dog3=6)   
    combined = combine_dataclass_instances(dog1, cat1)

    try:
        combined.meow()
    except Exception as e:
        print(e)

    
    combined.bark()
    print(asdict(combined))
    print(combined.__class__)
