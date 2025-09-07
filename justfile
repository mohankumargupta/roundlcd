set shell := ["sh", "-c"]
set windows-shell := ["powershell", "-c"]

compile:
    esphome compile roundlcd.yaml

latest:
    uv tool install esphome 
    uv tool install pip



