import { PrismaClient } from '@prisma/client';
import type { NextApiRequest, NextApiResponse } from 'next';

const prisma = new PrismaClient();

// This is a custom API route for handling onboarding
export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // In a real implementation, we would verify the user session here
    // For now, we'll simulate receiving the user ID from the frontend
    // In practice, you'd extract the user ID from a JWT token or session cookie
    const { userId, pythonProficiency, operatingSystem, preferredEnvironment } = req.body;

    // Validate required fields
    if (!userId || !pythonProficiency || !operatingSystem || !preferredEnvironment) {
      return res.status(400).json({ error: 'Missing required fields' });
    }

    // Create or update user profile
    const profile = await prisma.userProfile.upsert({
      where: {
        userId: userId,
      },
      update: {
        pythonProficiency,
        operatingSystem,
        preferredEnvironment,
        onboardingComplete: true,
      },
      create: {
        userId: userId,
        pythonProficiency,
        operatingSystem,
        preferredEnvironment,
        onboardingComplete: true,
      },
    });

    res.status(200).json({
      success: true,
      profile,
      redirectTo: '/dashboard' // Redirect to main content after onboarding
    });
  } catch (error) {
    console.error('Onboarding error:', error);
    res.status(500).json({ error: 'Failed to complete onboarding' });
  } finally {
    await prisma.$disconnect();
  }
}