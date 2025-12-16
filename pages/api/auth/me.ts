import { PrismaClient } from '@prisma/client';
import type { NextApiRequest, NextApiResponse } from 'next';

const prisma = new PrismaClient();

// This is a custom API route for getting current user info
export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // In a real implementation, we would verify the user session here
    // For now, we'll simulate receiving the user ID from the frontend
    // In practice, you'd extract the user ID from a JWT token or session cookie
    const authHeader = req.headers.authorization;

    if (!authHeader) {
      return res.status(401).json({ error: 'Unauthorized: No token provided' });
    }

    // In a real implementation with Better Auth, you would verify the JWT token
    // For now, we'll assume the header value is the user ID (this is just for demo purposes)
    const userId = authHeader;

    if (!userId) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    // Get user info from Better Auth (in a real implementation)
    // For now, we'll return mock data
    const user = {
      id: userId as string,
      email: 'user@example.com', // This would come from the session
      name: 'User Name', // This would come from the session
    };

    // Get user profile
    const profile = await prisma.userProfile.findUnique({
      where: {
        userId: userId as string,
      },
    });

    res.status(200).json({
      user,
      profile,
      onboardingComplete: profile?.onboardingComplete || false,
    });
  } catch (error) {
    console.error('Get current user error:', error);
    res.status(500).json({ error: 'Failed to get user info' });
  } finally {
    await prisma.$disconnect();
  }
}