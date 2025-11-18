# Compare Parallel Branch Implementations

Please perform a comprehensive file-by-file and code-block-by-code-block comparison of the different working branches for this feature and generate a detailed comparison markdown file.


---

## Analysis Requirements

**First, check if `BRANCH_COMPARISON_[feature_name].md` already exists:**
- If it exists and contains user preferences → Add a **Final Action:** section under each user preference box that is filled in, describing what you'll do based on their input (don't regenerate the file)
- If it doesn't exist → Proceed with full analysis below

1. **Identify All Branches**
   - List all relevant working branches being compared
   - Show brief description of each branch's focus

2. **File-Level Diff Comparison**
   - Organize by file path
   - For each file that differs, show what each branch implemented
   - Use "Not Implemented" if a branch didn't touch that file
   - Provide pros/cons/assessment for each file's implementations

3. **Code Block Level Analysis**
   - For significant files, break down into logical code blocks (classes, functions, major sections)
   - Compare implementations at this granular level
   - Show code snippets or detailed summaries for each branch

4. **Final Recommendation**
   - After all file comparisons, provide overall recommendation
   - State which parts to take from which branches
   - Propose better alternatives where appropriate

## Output Format

Generate a markdown file named `BRANCH_COMPARISON_[feature_name].md` with the following structure:

```markdown
# Branch Comparison: [Feature Name]

## Branches Analyzed
- **Branch A**: [name] - [brief description]
- **Branch B**: [name] - [brief description]
- **Branch C**: [name] - [brief description]

---

## File Comparisons

### File: `path/to/file1.py`

#### Branch A Implementation
```python
# Code snippet or detailed summary
class MyClass:
    def method(self):
        # implementation details
```
[Brief description of approach]

**Pros:**
- [bullet point]
- [bullet point]

**Cons:**
- [bullet point]

#### Branch B Implementation
```python
# Different approach
class MyClass:
    def method(self):
        # different implementation
```
[Brief description of approach]

**Pros:**
- [bullet point]

**Cons:**
- [bullet point]

#### Branch C Implementation
**Not Implemented** - This branch did not modify this file.

#### Assessment for `file1.py`
**Recommendation**: [Branch A / Branch B / Combined approach / New alternative]

**Rationale**: [Explain why this is the best approach for this specific file]

**Action**: [What to do - use Branch A as-is, modify Branch B, combine elements, rewrite, etc.]

```markdown
**Enter your preferences here:**:


```

---

### File: `path/to/file2.py`

#### Branch A Implementation
**Not Implemented**

#### Branch B Implementation
[Code summary or snippet]
[Description]

**Pros:**
- Feature X provides capability Y
- Clean implementation

**Cons:**
- None significant

#### Branch C Implementation
[Code summary or snippet]
[Description]

**Pros:**
- More extensible design
- Better error handling

**Cons:**
- More complex

#### Assessment for `file2.py`
**Recommendation**: [Choose one or combine]

**Rationale**: [Reasoning]

**Action**: [Specific next steps]

```markdown
**Enter your preferences here:**:


```

---

### File: `path/to/file3.py` - Code Block: `ClassName.method_name()`

[For complex files, break into code blocks]

#### Branch A Implementation
```python
def method_name(self, arg1, arg2):
    # approach A
```

**Pros:**
- [specific to this method]

**Cons:**
- [specific to this method]

#### Branch B Implementation
**Not Implemented** - Method doesn't exist in Branch B

#### Assessment for this Code Block
[Analysis specific to just this function/class/block]

---

[Repeat for all files and significant code blocks]

---

## Overall Recommendation

### Summary of Choices by File
| File | Recommended Source | Notes |
|------|-------------------|-------|
| `file1.py` | Branch A | Use as-is |
| `file2.py` | Branch C with modifications | Add error handling from Branch B |
| `file3.py` | New implementation | Combine ideas from A and C |

### Implementation Plan
1. Start with Branch [X] as the base
2. Cherry-pick `file1.py` from Branch [Y]
3. For `file2.py`: Take Branch [Z]'s approach but add [specific modification]
4. For `file3.py`: Create new implementation combining [specific elements]

### Rationale
[Overall justification for the combined approach]

### Files/Features to Exclude
- `bad_file.py` from Branch A - [reason it's bad]
- Feature X from Branch B - [reason to exclude]

## Next Steps
1. [Specific action item]
2. [Specific action item]

```markdown
**Enter your preferences here:**:


```

```
## Important Notes
- **Be granular**: Compare at file and code-block level, not just overall
- **Use code snippets**: Show actual code differences when relevant
- **Mark "Not Implemented"**: Clearly indicate when a branch didn't touch something
- **Assess each file separately**: Don't just give an overall assessment
- **Be specific**: "Use Branch A's approach for file X" not "Branch A is better"
- **Consider partial adoption**: You can recommend taking parts of different implementations
- **Ignore bad code**: If an implementation is poor, mark it and move on
