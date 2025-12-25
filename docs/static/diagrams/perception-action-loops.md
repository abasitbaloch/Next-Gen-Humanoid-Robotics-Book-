# Perception-Action Loop Diagrams

## Basic Perception-Action Loop

```mermaid
graph TD
    A[Environment State] --> B(Sensor Reading)
    B --> C{Perception<br/>Processing}
    C --> D[Internal State]
    D --> E{Action<br/>Planning}
    E --> F[Motor Command]
    F --> G[Robot Action]
    G --> A
    style A fill:#e1f5fe
    style G fill:#f3e5f5
    style C fill:#e8f5e8
    style E fill:#fff3e0
```

## Hierarchical Perception-Action Loop

```mermaid
graph TB
    subgraph "High-Level Loop"
        A1[Task Goal] --> B1{Task Planning}
        B1 --> C1[Subtask Sequence]
        C1 --> A1
    end

    subgraph "Mid-Level Loop"
        A2[Subtask] --> B2{Motion Planning}
        B2 --> C2[Motor Trajectory]
        C2 --> A2
    end

    subgraph "Low-Level Loop"
        A3[Sensory Feedback] --> B3{Motor Control}
        B3 --> C3[Joint Commands]
        C3 --> A3
    end

    A1 -.-> A2
    A2 -.-> A3
    C3 -.-> C2
    C2 -.-> C1
```

## Reactive vs Deliberative Systems

```mermaid
graph LR
    subgraph "Reactive System"
        A1[Sensor Input] --> B1{Simple<br/>Condition}
        B1 --> C1[Immediate<br/>Action]
        C1 --> A1
    end

    subgraph "Deliberative System"
        A2[Sensor Input] --> B2[World Model<br/>Update]
        B2 --> C2{Plan<br/>Generation}
        C2 --> D2[Plan<br/>Execution]
        D2 --> A2
    end
```

## Sensor Fusion in Perception-Action Loop

```mermaid
graph TD
    A[Environment] --> B[Camera]
    A --> C[LIDAR]
    A --> D[IMU]
    A --> E[Tactile Sensors]

    B --> F[Sensor Fusion]
    C --> F
    D --> F
    E --> F

    F --> G{State Estimation}
    G --> H[Robot State]
    H --> I{Action Selection}
    I --> J[Motor Commands]
    J --> A

    style B fill:#ffebee
    style C fill:#ffebee
    style D fill:#ffebee
    style E fill:#ffebee
    style F fill:#e8f5e8
    style I fill:#fff3e0
```

## Feedback Mechanisms

```mermaid
graph TD
    A[Desired State] --> B{Controller}
    B --> C[Control Signal]
    C --> D[Robot/Plant]
    D --> E[Actual State]
    E --> F{Sensor}
    F --> G[Measured State]
    G --> H{Comparator}
    H --> I[Error Signal]
    I --> B

    B -.-> J[Feedforward<br/>Compensation]
    D -.-> K[Disturbance<br/>Rejection]

    style A fill:#e3f2fd
    style E fill:#e8f5e8
    style D fill:#f3e5f5
```