import React, { useState } from 'react';

interface OnboardingData {
  pythonProficiency: string;
  operatingSystem: string;
  preferredEnvironment: string;
}

interface OnboardingQuestionnaireProps {
  onComplete: (data: OnboardingData) => void;
}

const OnboardingQuestionnaire: React.FC<OnboardingQuestionnaireProps> = ({ onComplete }) => {
  const [formData, setFormData] = useState<OnboardingData>({
    pythonProficiency: '',
    operatingSystem: '',
    preferredEnvironment: '',
  });
  const [error, setError] = useState('');

  const handleChange = (e: React.ChangeEvent<HTMLSelectElement | HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: value,
    }));
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();

    // Validate required fields
    if (!formData.pythonProficiency || !formData.operatingSystem || !formData.preferredEnvironment) {
      setError('Please answer all questions');
      return;
    }

    setError('');
    onComplete(formData);
  };

  return (
    <div className="onboarding-form">
      <h2>Tell us about your background</h2>
      <p>This will help us personalize your learning experience.</p>

      {error && <div className="error-message">{error}</div>}

      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="pythonProficiency">Python Proficiency:</label>
          <select
            id="pythonProficiency"
            name="pythonProficiency"
            value={formData.pythonProficiency}
            onChange={handleChange}
            required
          >
            <option value="">Select your level</option>
            <option value="BEGINNER">Beginner</option>
            <option value="INTERMEDIATE">Intermediate</option>
            <option value="EXPERT">Expert</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="operatingSystem">Operating System:</label>
          <select
            id="operatingSystem"
            name="operatingSystem"
            value={formData.operatingSystem}
            onChange={handleChange}
            required
          >
            <option value="">Select your OS</option>
            <option value="WINDOWS">Windows</option>
            <option value="MACOS">macOS</option>
            <option value="LINUX">Linux</option>
            <option value="OTHER">Other</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="preferredEnvironment">Preferred Environment:</label>
          <select
            id="preferredEnvironment"
            name="preferredEnvironment"
            value={formData.preferredEnvironment}
            onChange={handleChange}
            required
          >
            <option value="">Select your preference</option>
            <option value="LOCAL_MACHINE">Local Machine</option>
            <option value="CLOUD_ENVIRONMENT">Cloud Environment</option>
          </select>
        </div>

        <button type="submit">Complete Onboarding</button>
      </form>
    </div>
  );
};

export default OnboardingQuestionnaire;