from fastapi import FastAPI

app = FastAPI()


@app.get("/")
async def read_items():
    return "aero_planner"