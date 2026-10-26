---
hide:
  - toc
---

<div class="px4-home">
  <section class="px4-home-intro" aria-labelledby="px4-development-toolkit">
    <h1 id="px4-development-toolkit">PX4 Development Toolkit</h1>
    <p class="px4-home-lede">A development environment for PX4 and ROS 2, supporting controller experiments in Gazebo simulation and on physical aircraft.</p>
    <p class="px4-home-summary">Start with a simulation, choose where your controller runs, then record and analyze an experiment. Follow the operational guides at the terminal or work through the shared dynamics and geometric-control derivation.</p>
    <div class="px4-home-actions">
      <a href="installation/" class="md-button md-button--primary">Get started</a>
      <a href="controllers/overview/" class="px4-home-secondary">Explore controllers <span aria-hidden="true">→</span></a>
    </div>
  </section>

  <section class="px4-home-media" aria-label="Simulation and physical-flight illustrations">
    <div class="px4-home-media-grid">
      <figure class="px4-home-media-item">
        <div class="px4-home-media-frame px4-home-media-frame--sim">
          <img src="assets/simulation.gif" width="640" height="360" alt="Gazebo SITL quadrotor flying a circular trajectory">
        </div>
        <figcaption><strong>Gazebo SITL</strong><span>Circular trajectory in Gazebo SITL</span></figcaption>
      </figure>
      <figure class="px4-home-media-item">
        <div class="px4-home-media-frame px4-home-media-frame--lab">
          <img src="assets/experiment.gif" width="640" height="360" alt="Real quadrotor flying a circular trajectory in the laboratory">
        </div>
        <figcaption><strong>Physical experiment</strong><span>Circular trajectory in the laboratory</span></figcaption>
      </figure>
    </div>
  </section>

  <!-- Preserve links to sections from the earlier documentation homepage. -->
  <span id="choose-an-installation"></span>
  <span id="suggested-first-journey"></span>
  <span id="start-here"></span>
  <span id="from-simulation-to-hardware"></span>
  <span id="simulation-and-hardware"></span>
  <span id="from-an-idea-to-a-recorded-flight"></span>
  <span id="understand-what-each-controller-calculates"></span>
  <span id="controller-interfaces"></span>

  <section class="px4-home-docs" aria-labelledby="documentation">
    <h2 id="documentation">Documentation</h2>
    <nav class="px4-home-directory" aria-label="Documentation sections">
      <a href="installation/" class="px4-home-doc-link"><span class="px4-home-doc-title">Get started <span aria-hidden="true">↗</span></span><span class="px4-home-doc-detail">Installation and first simulation</span></a>
      <a href="operations/tools/" class="px4-home-doc-link"><span class="px4-home-doc-title">Using the toolkit <span aria-hidden="true">↗</span></span><span class="px4-home-doc-detail">SITL, companion setup, onboard and Offboard workflows</span></a>
      <a href="controllers/overview/" class="px4-home-doc-link"><span class="px4-home-doc-title">Controllers <span aria-hidden="true">↗</span></span><span class="px4-home-doc-detail">PX4 reference, Offboard, and onboard controllers</span></a>
      <a href="mathematics/geometric-control/" class="px4-home-doc-link"><span class="px4-home-doc-title">Mathematics <span aria-hidden="true">↗</span></span><span class="px4-home-doc-detail">Frames, rigid-body dynamics, and geometric SE3 control</span></a>
      <a href="experiments/complete-example/" class="px4-home-doc-link"><span class="px4-home-doc-title">Experiments <span aria-hidden="true">↗</span></span><span class="px4-home-doc-detail">Define motion, record data, and compare observed behavior</span></a>
      <a href="reference/architecture/" class="px4-home-doc-link"><span class="px4-home-doc-title">Reference <span aria-hidden="true">↗</span></span><span class="px4-home-doc-detail">Architecture, configuration, and troubleshooting</span></a>
    </nav>
  </section>
</div>
