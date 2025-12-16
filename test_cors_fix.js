// Test script to simulate frontend communication with backend
// This simulates what the frontend would do when calling the backend API

console.log("Testing frontend-backend communication...");

// Simulate the API call that the frontend would make
const testAPICommunication = async () => {
    try {
        console.log("Making request to: http://localhost:8000/api/ask");

        // Test the backend API with correct endpoint
        const response = await fetch('http://localhost:8000/api/ask', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                question: 'Test question from frontend',
                mode: 'full_book'
            })
        });

        if (!response.ok) {
            throw new Error(`API request failed with status ${response.status}`);
        }

        const data = await response.json();
        console.log("Success! Received response from backend:");
        console.log(data);
        console.log("\nCORS communication is working correctly!");
        console.log("The frontend can now successfully communicate with the backend.");

        return data;
    } catch (error) {
        console.error("Error in API communication:", error);
        throw error;
    }
};

// Run the test
testAPICommunication()
    .then(() => console.log("\nTest completed successfully!"))
    .catch(error => console.error("Test failed:", error));