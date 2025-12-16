import { PrismaClient } from '@prisma/client';
import type { NextApiRequest, NextApiResponse } from 'next';

const prisma = new PrismaClient();

// This is a custom API route for updating user profile
export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'PUT' && req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // Extract token from Authorization header
    const authHeader = req.headers.authorization;

    if (!authHeader) {
      return res.status(401).json({ error: 'Unauthorized: No token provided' });
    }

    // In a real implementation with Better Auth, you would verify the JWT token
    // For now, we'll assume the header value is the user ID (this is just for demo purposes)
    // In a real app, you would decode the JWT token to get the user ID
    const userId = authHeader;

    if (!userId) {
      return res.status(401).json({ error: 'Unauthorized: Invalid token' });
    }

    if (req.method === 'GET') {
      // Get user profile
      const profile = await prisma.userProfile.findUnique({
        where: {
          userId: userId as string,
        },
      });

      res.status(200).json({
        profile,
        onboardingComplete: profile?.onboardingComplete || false,
      });
    } else if (req.method === 'PUT') {
      // Update user profile
      const { pythonProficiency, operatingSystem, preferredEnvironment, name } = req.body;

      // Validate required fields
      if (!pythonProficiency || !operatingSystem || !preferredEnvironment) {
        return res.status(400).json({ error: 'Missing required profile fields' });
      }

      // Update user profile
      const profile = await prisma.userProfile.upsert({
        where: {
          userId: userId as string,
        },
        update: {
          pythonProficiency,
          operatingSystem,
          preferredEnvironment,
          onboardingComplete: true, // User is updating profile, so onboarding is complete
        },
        create: {
          userId: userId as string,
          pythonProficiency,
          operatingSystem,
          preferredEnvironment,
          onboardingComplete: true,
        },
      });

      // Update user name if provided
      if (name) {
        // In a real implementation, you would update the user's name in the Better Auth database
        // For now, we'll just log this
        console.log(`User ${userId} updated their name to ${name}`);
      }

      res.status(200).json({
        success: true,
        profile,
      });
    }
  } catch (error) {
    console.error('Profile update error:', error);
    res.status(500).json({ error: 'Failed to update profile' });
  } finally {
    await prisma.$disconnect();
  }
}