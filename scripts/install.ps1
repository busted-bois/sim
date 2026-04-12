Write-Host "Setting up venv..."
uv sync
uv run lefthook install

Write-Host "Setup complete."
