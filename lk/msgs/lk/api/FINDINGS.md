# ClientResponse Type Hinting - Key Findings

## Problem Statement
We have a `ClientResponse[T]` wrapper that delegates attribute access to wrapped data via `__getattr__`. The challenge: how to get IDE autocomplete for both the wrapper's attributes (`.header`, `.success`) AND the delegated data attributes (`.reward`, `.state`) without sacrificing API ergonomics?

## Solution: Union Type Hint `ClientResponse[T] | T`

**The Discovery:** Using `ClientResponse[T] | T` as the return type provides autocomplete for BOTH types!

```python
def step(self, action: Action) -> ClientResponse[Observation] | Observation:
    obs = Observation()
    obs.reward = 1.0
    obs.state = np.zeros(4)
    return ClientResponse.success(data=obs)

# Usage - ALL of these get full autocomplete! 🎉
response = env.step(action)
response.reward        # ✓ Autocomplete from Observation
response.state         # ✓ Autocomplete from Observation  
response.header        # ✓ Autocomplete from ClientResponse
response.success       # ✓ Autocomplete from ClientResponse
response.reward = 100  # ✓ Can modify attributes
```

## Why This Works

1. **Type Checker Behavior**: Python type checkers (Pylance, mypy) provide autocomplete for attributes from ANY type in a union
2. **Runtime Behavior**: `ClientResponse.__getattr__()` transparently delegates missing attributes to `.data`
3. **Result**: You get autocomplete for BOTH `ClientResponse` and `T` attributes, and everything works at runtime!

## Comparison with Other Approaches

### ❌ Approach 1: `ClientResponse[T]` (Standard)
```python
def step(self) -> ClientResponse[Observation]:
    ...

response = env.step(action)
response.reward        # ✗ No autocomplete (works at runtime)
response.data.reward   # ✓ Autocomplete works
response.header        # ✓ Autocomplete works
```
**Rating:** ★★★☆☆ - Works but requires `.data` for autocomplete

### ✅ Approach 2: `ClientResponse[T] | T` (RECOMMENDED)
```python
def step(self) -> ClientResponse[Observation] | Observation:
    ...

response = env.step(action)
response.reward        # ✓ Autocomplete works!
response.header        # ✓ Autocomplete works!
```
**Rating:** ★★★★★ - Best of both worlds!

### ⚠️ Approach 3: `.as_type()` Helper
```python
def step(self) -> ClientResponse[Observation]:
    ...

response = env.step(action)
obs = response.as_type()  # Cast for type checker
obs.reward        # ✓ Autocomplete works
obs.header        # ? Type checker might complain
```
**Rating:** ★★★★☆ - Works but requires extra method call

### ⚠️ Approach 4: Document Runtime-Only Delegation
```python
def step(self) -> ClientResponse[Observation]:
    """Returns ClientResponse. Access .data for IDE support."""
    ...

response = env.step(action)
response.reward        # ✗ No autocomplete (works at runtime)
response.data.reward   # ✓ Autocomplete works
```
**Rating:** ★★★☆☆ - Requires discipline

## Implementation Requirements

For this to work, `ClientResponse` must:

1. **Implement `__getattr__`** to delegate to wrapped data:
```python
def __getattr__(self, name: str):
    return getattr(self.data, name)
```

2. **Implement `__setattr__`** to delegate attribute setting:
```python
def __setattr__(self, name: str, value):
    if name in ['data', 'header', 'success', ...]:  # Internal attrs
        super().__setattr__(name, value)
    else:
        setattr(self.data, name, value)
```

3. **Use union return type** in function signatures:
```python
def my_function() -> ClientResponse[MyType] | MyType:
    return ClientResponse.success(data=MyType())
```

4. **Automatic namespace conflict detection** - warns if data attributes conflict with wrapper:
```python
def _check_namespace_conflicts(self) -> None:
    """Automatically detect conflicts by introspecting ClientResponse class."""
    # Automatically get all public attributes from ClientResponse
    reserved_attrs = {attr for attr in dir(ClientResponse) if not attr.startswith('_')}
    reserved_attrs.update(['data', 'header', 'info'])  # Instance attrs
    
    # Check for conflicts with data attributes
    data_attrs = {attr for attr in dir(self.data) if not attr.startswith('_')}
    conflicts = reserved_attrs & data_attrs
    
    if conflicts:
        warnings.warn(f"Namespace conflict: {conflicts}. Use .data.{...}")
```

## Trade-offs

### Potential Namespace Conflicts

⚠️ **The union type approach means data attributes could conflict with ClientResponse attributes**

**Example conflict:**
```python
class MyData:
    header: str = "my custom header"  # Conflicts with ClientResponse.header!

response = ClientResponse.success(data=MyData())
response.header  # Returns ResponseHeader (ClientResponse wins)
response.data.header  # Returns "my custom header" (explicit access)
```

**Solution:** Automatic conflict detection warns you at runtime, and you can always use `.data.attribute` for explicit disambiguation.

## Benefits

✅ **Full IDE Autocomplete** - for both wrapper and data attributes  
✅ **Clean Runtime API** - no need for `.data` or `.as_type()`  
✅ **Type Safety** - type checker knows all available attributes  
✅ **Ergonomic** - write `response.reward` not `response.data.reward`  
✅ **Flexible** - can access wrapper features and data seamlessly  

## Recommendation

**Use `ClientResponse[T] | T` everywhere** as the return type for functions that return `ClientResponse`. This provides the best developer experience with full autocomplete and clean API.

---

*Discovered during pair programming session, November 2025*

