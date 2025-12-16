import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

export class UserProfileService {
  /**
   * Create or update user profile
   */
  async createOrUpdateProfile(
    userId: string,
    profileData: {
      pythonProficiency: string;
      operatingSystem: string;
      preferredEnvironment: string;
      onboardingComplete?: boolean;
    }
  ) {
    try {
      const profile = await prisma.userProfile.upsert({
        where: {
          userId: userId,
        },
        update: {
          pythonProficiency: profileData.pythonProficiency,
          operatingSystem: profileData.operatingSystem,
          preferredEnvironment: profileData.preferredEnvironment,
          onboardingComplete: profileData.onboardingComplete ?? true,
        },
        create: {
          userId: userId,
          pythonProficiency: profileData.pythonProficiency,
          operatingSystem: profileData.operatingSystem,
          preferredEnvironment: profileData.preferredEnvironment,
          onboardingComplete: profileData.onboardingComplete ?? true,
        },
      });

      return profile;
    } catch (error) {
      throw new Error(`Failed to create or update user profile: ${error}`);
    }
  }

  /**
   * Get user profile by user ID
   */
  async getProfileByUserId(userId: string) {
    try {
      const profile = await prisma.userProfile.findUnique({
        where: {
          userId: userId,
        },
      });

      return profile;
    } catch (error) {
      throw new Error(`Failed to fetch user profile: ${error}`);
    }
  }

  /**
   * Check if user has completed onboarding
   */
  async isOnboardingComplete(userId: string): Promise<boolean> {
    try {
      const profile = await prisma.userProfile.findUnique({
        where: {
          userId: userId,
        },
      });

      return profile ? profile.onboardingComplete : false;
    } catch (error) {
      throw new Error(`Failed to check onboarding status: ${error}`);
    }
  }

  /**
   * Update onboarding completion status
   */
  async updateOnboardingStatus(userId: string, status: boolean) {
    try {
      const profile = await prisma.userProfile.upsert({
        where: {
          userId: userId,
        },
        update: {
          onboardingComplete: status,
        },
        create: {
          userId: userId,
          pythonProficiency: "BEGINNER", // default value
          operatingSystem: "OTHER", // default value
          preferredEnvironment: "LOCAL_MACHINE", // default value
          onboardingComplete: status,
        },
      });

      return profile;
    } catch (error) {
      throw new Error(`Failed to update onboarding status: ${error}`);
    }
  }
}

// Export a singleton instance
export const userProfileService = new UserProfileService();