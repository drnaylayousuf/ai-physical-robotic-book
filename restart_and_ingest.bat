@echo off
echo Stopping all Python processes...

REM Kill all Python processes
tasklist /FI "IMAGENAME eq python.exe" 2>NUL | find /I /N "python.exe">NUL
if "%ERRORLEVEL%"=="0" (
    echo Killing Python processes...
    taskkill /F /IM python.exe 2>NUL
) else (
    echo No Python processes found.
)

REM Wait a moment for processes to terminate
timeout /T 3 /NOBREAK >NUL

echo Starting the backend server...
start /min python -c "import uvicorn; from backend.main import app; uvicorn.run(app, host='0.0.0.0', port=8000)"

REM Wait for server to start
echo Waiting for server to start...
timeout /T 10 /NOBREAK >NUL

REM Run the ingestion
echo Running ingestion...
python -c "
import requests
import os
from datetime import timedelta
from backend.config.settings import settings
from backend.utils.auth import create_access_token, create_token_data
from backend.models.database import SessionLocal
from backend.models.user import User

def generate_admin_token():
    db = SessionLocal()
    try:
        admin_user = db.query(User).filter(User.username == 'admin').first()
        if not admin_user:
            raise Exception('Admin user not found in database')
        token_data = create_token_data(admin_user.username, admin_user.role)
        token = create_access_token(data=token_data, expires_delta=timedelta(minutes=60))
        return token
    finally:
        db.close()

# Wait for server to be ready
for i in range(10):
    try:
        response = requests.get('http://localhost:8000/api/health', timeout=5)
        if response.status_code == 200:
            print('Server is ready!')
            break
    except:
        print(f'Waiting for server... ({i+1}/10)')
        import time
        time.sleep(2)
else:
    print('Server failed to start')
    exit(1)

# Generate admin token
admin_token = generate_admin_token()
print('Token generated')

# Call ingestion endpoint
headers = {
    'Authorization': f'Bearer {admin_token}',
    'Content-Type': 'application/json'
}

source_path = os.path.abspath('doc')
ingestion_data = {
    'source_path': source_path,
    'collection_name': 'book_chunks'
}

try:
    print('Calling ingestion endpoint...')
    response = requests.post(
        'http://localhost:8000/api/ingest',
        headers=headers,
        json=ingestion_data,
        timeout=300
    )
    print(f'Ingestion response status: {response.status_code}')
    print(f'Ingestion response: {response.text}')

    if response.status_code == 200:
        print('Ingestion completed successfully!')
    else:
        print(f'Ingestion failed with status {response.status_code}')

    # Test diagnostic after ingestion
    diag_response = requests.get('http://localhost:8000/api/diagnostic/qdrant')
    print(f'Diagnostic response: {diag_response.text}')

    # Test QA after ingestion
    qa_response = requests.post('http://localhost:8000/api/ask',
                              json={'question': 'What is humanoid robotics?', 'mode': 'full_book'},
                              headers={'Content-Type': 'application/json'})
    print(f'QA test response: {qa_response.text}')

except requests.exceptions.RequestException as e:
    print(f'Error calling ingestion endpoint: {e}')
"