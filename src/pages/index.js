import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro/overview">
            Read the Book 
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="Building Embodied Intelligent Systems">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h2>ROS 2 Fundamentals</h2>
                <p>Learn the Robot Operating System 2 for building robust robot applications.</p>
              </div>
              <div className="col col--4">
                <h2>Simulation & Control</h2>
                <p>Master Gazebo, Unity, and NVIDIA Isaac for robot simulation and control.</p>
              </div>
              <div className="col col--4">
                <h2>Vision-Language-Action</h2>
                <p>Discover LLM-driven robotics and multimodal AI systems.</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}