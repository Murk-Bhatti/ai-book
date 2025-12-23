import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

const ModuleList = [
  {
    title: 'Module 1: ROS 2 Foundations',
    description: (
      <>
        Master the Robot Operating System 2 - the nervous system of modern robots.
        Learn nodes, topics, services, and actions for building robust robot software.
      </>
    ),
    link: '/docs/module-1-ros2/introduction',
    chapters: 4,
  },
  {
    title: 'Module 2: Digital Twin',
    description: (
      <>
        Build virtual replicas of physical robots using Gazebo and Unity.
        Simulate physics, sensors, and environments for safe development and testing.
      </>
    ),
    link: '/docs/module-2-digital-twin/introduction',
    chapters: 4,
  },
  {
    title: 'Module 3: NVIDIA Isaac',
    description: (
      <>
        Leverage GPU-accelerated perception and navigation with NVIDIA Isaac.
        Deploy AI models for visual SLAM, object detection, and autonomous navigation.
      </>
    ),
    link: '/docs/module-3-nvidia-isaac/introduction',
    chapters: 4,
  },
  {
    title: 'Module 4: Vision-Language-Action',
    description: (
      <>
        Connect natural language to robot actions using VLA architectures.
        Integrate speech recognition, LLM planning, and end-to-end autonomous systems.
      </>
    ),
    link: '/docs/module-4-vla/introduction',
    chapters: 4,
  },
];

function Module({title, description, link, chapters}) {
  return (
    <div className={clsx('col col--6', styles.moduleCol)}>
      <div className={styles.moduleCard}>
        <div className="padding--md">
          <Heading as="h3">{title}</Heading>
          <p>{description}</p>
          <div className={styles.moduleFooter}>
            <span className={styles.chapterCount}>{chapters} Chapters</span>
            <Link className="button button--primary button--sm" to={link}>
              Read Module
            </Link>
          </div>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="text--center margin-bottom--lg">
          <Heading as="h2">What You Will Learn</Heading>
          <p className={styles.sectionSubtitle}>
            Four comprehensive modules taking you from ROS 2 basics to building
            autonomous humanoid robots with AI capabilities.
          </p>
        </div>
        <div className="row">
          {ModuleList.map((props, idx) => (
            <Module key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
