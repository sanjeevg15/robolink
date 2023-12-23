import { useEffect, useRef, useState } from "react";
import "./App.css";

function App() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const contextRef = useRef<CanvasRenderingContext2D>(null);
  // const contextRef = useRef(null);

  const [jointA, setJointA] = useState([0, 0]);
  const [jointB, setJointB] = useState([0, 0]);
  const [jointC, setJointC] = useState([0, 0]);
  const link_img = new Image();
  // link_img.src = "./src/assets/robot_arm.png";
  link_img.src = "./src/assets/robot_arm.svg";

  const reset = () => {
    const canvas = canvasRef.current;
    if (!canvas) {
      console.error("Canvas element not found");
      return;
    }
    const context = canvas.getContext("2d");
    if (!context) {
      console.error("2D Context not available");
      return;
    }
    context.clearRect(0, 0, canvas.width, canvas.height);
    let url = "http://localhost:5000/resetManipulator";
    fetch(url, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        screen_width: canvas.width,
        screen_height: canvas.height,
        // screen_width: window.innerWidth,
        // screen_height: window.innerHeight,
      }),
    })
      .then((response) => response.json())
      .then((jsonData) => {
        setJointA(jsonData.joint_positions[1]);
        setJointB(jsonData.joint_positions[2]);
        setJointC(jsonData.joint_positions[3]);
      });
  };

  const renderManipulator = () => {
    const canvas = canvasRef.current;
    if (!canvas) {
      console.error("Canvas element not found");
      return;
    }

    // console.log("Joint A", jointA);
    // console.log("Joint B", jointB);
    // console.log("Joint C", jointC);
    const context = canvas.getContext("2d");
    if (!context) {
      console.error("2D Context not available");
      return;
    }

    context.clearRect(0, 0, canvas.width, canvas.height);
    context.moveTo(canvas.width / 2, canvas.height);
    context.beginPath();
    context.strokeStyle = "blue";
    context.lineWidth = 3;
    context.arc(canvas.width / 2, canvas.height, 300, 0, Math.PI, true);
    context.stroke();
    // context.closePath();

    // context.drawImage(link_img, 200, 200, 30, 150);
    context.drawImage(
      link_img,
      canvas.width / 2 - 15,
      canvas.height - 150,
      30,
      150
    );
    context.moveTo(canvas.width / 2, canvas.height);
    // context.beginPath();
    context.lineCap = "round";
    context.strokeStyle = "black";
    context.lineWidth = 10;
    context.lineJoin = "round";

    context.lineTo(jointA[0], jointA[1]);
    // context.moveTo(jointA[0], jointA[1]);
    context.stroke();
    context.arc(jointA[0], jointA[1], 6, 0, Math.PI * 2);
    context.stroke();
    context.lineTo(jointB[0], jointB[1]);
    context.stroke();
    context.arc(jointB[0], jointB[1], 6, 0, Math.PI * 2);
    context.stroke();
    context.lineTo(jointC[0], jointC[1]);
    context.stroke();
    context.closePath();

    contextRef.current = context;
  };
  useEffect(renderManipulator, [jointA, jointB, jointC]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      console.log("Canvas element not found");
      return;
    }
    const context = canvas.getContext("2d");
    if (!context) {
      console.error("2D Context not available");
      return;
    }

    const scale_factor = 0.9;

    context.lineWidth = 5;
    canvas.height = window.innerHeight * scale_factor;
    canvas.width = window.innerWidth * scale_factor;
    canvas.style.width = `${window.innerWidth * scale_factor}px`;
    canvas.style.height = `${window.innerHeight * scale_factor}px`;

    context.lineCap = "round";
    context.strokeStyle = "black";
    context.lineWidth = 5;
    contextRef.current = context;
  }, []);

  const startDrawing = (nativeEvent: React.MouseEvent<HTMLCanvasElement>) => {
    const { pageX, pageY } = nativeEvent;
    const canvas = canvasRef.current;
    const canvasPosition = canvas?.getBoundingClientRect();
    if (!canvasPosition) {
      return;
    }
    const [canvasX, canvasY] = [
      pageX - canvasPosition?.left,
      pageY - canvasPosition?.top,
    ];
    const context = contextRef.current;
    if (!context) {
      return;
    }
    context.moveTo(canvasX, canvasY);
    context.beginPath();
    context.arc(canvasX, canvasY, 6, 0, 2 * Math.PI);
    context.fillStyle = "blue"; // Set the circle's fill color
    context.fill();
    context.closePath();
    // context?.fillRect(canvasX, canvasY, 10, 10);
  };

  const postData = (nativeEvent: React.MouseEvent<HTMLCanvasElement>) => {
    try {
      // const [offsetX, offsetY] = [nativeEvent.clientX, nativeEvent.clientY];
      const canvas = canvasRef.current;
      if (!canvas) {
        return;
      }
      const { pageX, pageY } = nativeEvent;
      const canvasPosition = canvas?.getBoundingClientRect();
      const [canvasX, canvasY] = [
        pageX - canvasPosition?.left,
        pageY - canvasPosition?.top,
      ];
      // const [canvasX, canvasY] = [pageX, pageY];

      const url = "http://localhost:5000/getJointPositions";
      fetch(url, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          target: [canvasX, canvasY],
          screen_width: canvas.width,
          screen_height: canvas.height,
        }),
      })
        .then((res) => res.json())
        .then((jsonData) => {
          const status = jsonData["status"];
          console.log("Status: ", status);
          if (status != "Reachable") {
            console.log("Unreachable");
          } else {
            const joint_positions = jsonData["joint_positions"];
            console.log("Joint Positons: ", joint_positions);
            joint_positions.map((curr_joint_positions: any) => {
              setJointA(curr_joint_positions[1]);
              setJointB(curr_joint_positions[2]);
              setJointC(curr_joint_positions[3]);
            });
          }
        });

      // setMessage(response["message"]); // Update state with the response message
    } catch (error) {
      console.log("Error:", error);
    }
  };

  return (
    <div>
      <canvas
        id="canvas"
        ref={canvasRef}
        onMouseDown={(nativeEvent) => {
          postData(nativeEvent);
          startDrawing(nativeEvent);
        }}
      ></canvas>
      <button onClick={reset}> Start </button>
      <input title="Algorithm" type="radio" name="form1" /> Box Search
      <input title="Algorithm" type="radio" name="form1" /> Gradient Descent
    </div>
  );
}

export default App;
