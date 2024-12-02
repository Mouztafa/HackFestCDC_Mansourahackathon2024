document.addEventListener("DOMContentLoaded", () => {
  const form = document.getElementById("medicine-form");
  const reminderList = document.getElementById("reminder-list");
  const alarmSound = document.getElementById("alarm-sound");

  const reminders = [];

  form.addEventListener("submit", (e) => {
    e.preventDefault();

    const medicineName = document.getElementById("medicine-name").value;
    const time = document.getElementById("time").value;

    reminders.push({ medicineName, time });
    displayReminders();

    form.reset();
  });

  function displayReminders() {
    reminderList.innerHTML = "";
    reminders.forEach((reminder) => {
      const li = document.createElement("li");
      li.textContent = `${reminder.medicineName} at ${reminder.time}`;
      reminderList.appendChild(li);
    });
  }

  function checkReminders() {
    const currentTime = new Date().toTimeString().slice(0, 5);
    reminders.forEach((reminder) => {
      if (reminder.time === currentTime) {
        alert(`Time to take your medicine: ${reminder.medicineName}`);
        alarmSound.play();
      }
    });
  }

  setInterval(checkReminders, 60000);
});
