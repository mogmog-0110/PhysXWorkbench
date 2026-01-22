import codecs

# Read the original file
with codecs.open(r'C:\Users\KAWAMURA Yuya\dev\PhysXSample\src\dx12\DX12Renderer.cpp', 'r', encoding='utf-8-sig') as f:
    lines = f.readlines()

# Read the new ImGui code
with open(r'C:\Users\KAWAMURA Yuya\dev\PhysXSample\new_imgui.txt', 'r', encoding='utf-8') as f:
    new_imgui = f.read()

# Find the start and end of the RenderImGui function
start_line = None
end_line = None

for i, line in enumerate(lines):
    if 'void DX12Renderer::RenderImGui' in line:
        start_line = i
    if start_line is not None and end_line is None:
        if line.strip() == '}' and i > start_line + 10:
            # Check if this is the closing brace of the function by looking ahead
            if i + 1 < len(lines) and (lines[i+1].strip().startswith('//===') or
                                        lines[i+1].strip().startswith('void ') or
                                        lines[i+1].strip() == ''):
                end_line = i
                break

print(f'Found RenderImGui function from line {start_line + 1} to {end_line + 1}')

# Replace the function
new_lines = lines[:start_line] + [new_imgui + '\n'] + lines[end_line + 1:]

# Write the modified file
with codecs.open(r'C:\Users\KAWAMURA Yuya\dev\PhysXSample\src\dx12\DX12Renderer.cpp', 'w', encoding='utf-8-sig') as f:
    f.writelines(new_lines)

print('Successfully replaced RenderImGui function!')
