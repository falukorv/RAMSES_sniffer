namespace RamsesSniffer
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.listBox1 = new System.Windows.Forms.ListBox();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.comboBox1 = new System.Windows.Forms.ComboBox();
            this.label5 = new System.Windows.Forms.Label();
            this.button1 = new System.Windows.Forms.Button();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.comboBox2 = new System.Windows.Forms.ComboBox();
            this.label11 = new System.Windows.Forms.Label();
            this.GPSTestCheckBox = new System.Windows.Forms.CheckBox();
            this.attitudeTestCheckBox = new System.Windows.Forms.CheckBox();
            this.gLoadTestCheckBox = new System.Windows.Forms.CheckBox();
            this.aRateTestCheckBox = new System.Windows.Forms.CheckBox();
            this.testTypeLabel = new System.Windows.Forms.Label();
            this.IIPTestCheckbox = new System.Windows.Forms.CheckBox();
            this.backgroundWorker1 = new System.ComponentModel.BackgroundWorker();
            this.GUIUpdateChecked = new System.Windows.Forms.CheckBox();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            // 
            // listBox1
            // 
            this.listBox1.FormattingEnabled = true;
            this.listBox1.HorizontalScrollbar = true;
            this.listBox1.Location = new System.Drawing.Point(12, 104);
            this.listBox1.Name = "listBox1";
            this.listBox1.Size = new System.Drawing.Size(790, 498);
            this.listBox1.TabIndex = 0;
            this.listBox1.SelectedIndexChanged += new System.EventHandler(this.listBox1_SelectedIndexChanged);
            // 
            // timer1
            // 
            this.timer1.Enabled = true;
            this.timer1.Interval = 50;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.Location = new System.Drawing.Point(10, 29);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(112, 25);
            this.label1.TabIndex = 1;
            this.label1.Text = "-00;00;00";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(41, 16);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(44, 13);
            this.label2.TabIndex = 2;
            this.label2.Text = "RT time";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(220, 9);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(32, 13);
            this.label3.TabIndex = 3;
            this.label3.Text = "GPS:";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(220, 22);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(35, 13);
            this.label4.TabIndex = 4;
            this.label4.Text = "label4";
            // 
            // comboBox1
            // 
            this.comboBox1.FormattingEnabled = true;
            this.comboBox1.Location = new System.Drawing.Point(13, 22);
            this.comboBox1.Name = "comboBox1";
            this.comboBox1.Size = new System.Drawing.Size(119, 21);
            this.comboBox1.TabIndex = 5;
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(12, 6);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(120, 13);
            this.label5.TabIndex = 6;
            this.label5.Text = "Sniffer network adapter:";
            this.label5.Click += new System.EventHandler(this.label5_Click);
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(138, 30);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(59, 30);
            this.button1.TabIndex = 7;
            this.button1.Text = "Start";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Location = new System.Drawing.Point(669, 12);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(133, 68);
            this.groupBox1.TabIndex = 8;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "POSNET";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(220, 44);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(32, 13);
            this.label6.TabIndex = 10;
            this.label6.Text = "GCS:";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(220, 57);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(35, 13);
            this.label7.TabIndex = 11;
            this.label7.Text = "label7";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(12, 88);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(35, 13);
            this.label8.TabIndex = 12;
            this.label8.Text = "label8";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(220, 88);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(35, 13);
            this.label9.TabIndex = 13;
            this.label9.Text = "label9";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(448, 9);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(48, 13);
            this.label10.TabIndex = 14;
            this.label10.Text = "GPS IIP:";
            // 
            // comboBox2
            // 
            this.comboBox2.FormattingEnabled = true;
            this.comboBox2.Location = new System.Drawing.Point(808, 125);
            this.comboBox2.Name = "comboBox2";
            this.comboBox2.Size = new System.Drawing.Size(121, 21);
            this.comboBox2.TabIndex = 15;
            this.comboBox2.SelectedIndexChanged += new System.EventHandler(this.comboBox2_SelectedIndexChanged);
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Enabled = false;
            this.label11.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F);
            this.label11.Location = new System.Drawing.Point(808, 104);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(53, 20);
            this.label11.TabIndex = 16;
            this.label11.Text = "Mode ";
            this.label11.UseWaitCursor = true;
            // 
            // GPSTestCheckBox
            // 
            this.GPSTestCheckBox.AutoSize = true;
            this.GPSTestCheckBox.Location = new System.Drawing.Point(808, 171);
            this.GPSTestCheckBox.Name = "GPSTestCheckBox";
            this.GPSTestCheckBox.Size = new System.Drawing.Size(48, 17);
            this.GPSTestCheckBox.TabIndex = 17;
            this.GPSTestCheckBox.Text = "GPS";
            this.GPSTestCheckBox.UseVisualStyleBackColor = true;
            // 
            // attitudeTestCheckBox
            // 
            this.attitudeTestCheckBox.AutoSize = true;
            this.attitudeTestCheckBox.Location = new System.Drawing.Point(808, 194);
            this.attitudeTestCheckBox.Name = "attitudeTestCheckBox";
            this.attitudeTestCheckBox.Size = new System.Drawing.Size(62, 17);
            this.attitudeTestCheckBox.TabIndex = 18;
            this.attitudeTestCheckBox.Text = "Attitude";
            this.attitudeTestCheckBox.UseVisualStyleBackColor = true;
            // 
            // gLoadTestCheckBox
            // 
            this.gLoadTestCheckBox.AutoSize = true;
            this.gLoadTestCheckBox.Location = new System.Drawing.Point(808, 217);
            this.gLoadTestCheckBox.Name = "gLoadTestCheckBox";
            this.gLoadTestCheckBox.Size = new System.Drawing.Size(62, 17);
            this.gLoadTestCheckBox.TabIndex = 19;
            this.gLoadTestCheckBox.Text = "G-loads";
            this.gLoadTestCheckBox.UseVisualStyleBackColor = true;
            // 
            // aRateTestCheckBox
            // 
            this.aRateTestCheckBox.AutoSize = true;
            this.aRateTestCheckBox.Location = new System.Drawing.Point(808, 240);
            this.aRateTestCheckBox.Name = "aRateTestCheckBox";
            this.aRateTestCheckBox.Size = new System.Drawing.Size(88, 17);
            this.aRateTestCheckBox.TabIndex = 20;
            this.aRateTestCheckBox.Text = "Angular rates";
            this.aRateTestCheckBox.UseVisualStyleBackColor = true;
            // 
            // testTypeLabel
            // 
            this.testTypeLabel.AutoSize = true;
            this.testTypeLabel.Location = new System.Drawing.Point(809, 152);
            this.testTypeLabel.Name = "testTypeLabel";
            this.testTypeLabel.Size = new System.Drawing.Size(156, 13);
            this.testTypeLabel.TabIndex = 21;
            this.testTypeLabel.Text = "What data do you want to test?";
            // 
            // IIPTestCheckbox
            // 
            this.IIPTestCheckbox.AutoSize = true;
            this.IIPTestCheckbox.Location = new System.Drawing.Point(808, 264);
            this.IIPTestCheckbox.Name = "IIPTestCheckbox";
            this.IIPTestCheckbox.Size = new System.Drawing.Size(39, 17);
            this.IIPTestCheckbox.TabIndex = 22;
            this.IIPTestCheckbox.Text = "IIP";
            this.IIPTestCheckbox.UseVisualStyleBackColor = true;
            // 
            // GUIUpdateChecked
            // 
            this.GUIUpdateChecked.AutoSize = true;
            this.GUIUpdateChecked.Checked = true;
            this.GUIUpdateChecked.CheckState = System.Windows.Forms.CheckState.Checked;
            this.GUIUpdateChecked.Cursor = System.Windows.Forms.Cursors.Default;
            this.GUIUpdateChecked.Location = new System.Drawing.Point(15, 49);
            this.GUIUpdateChecked.Name = "GUIUpdateChecked";
            this.GUIUpdateChecked.Size = new System.Drawing.Size(83, 17);
            this.GUIUpdateChecked.TabIndex = 23;
            this.GUIUpdateChecked.Text = "Update GUI";
            this.GUIUpdateChecked.UseVisualStyleBackColor = true;
            this.GUIUpdateChecked.CheckedChanged += new System.EventHandler(this.checkBox1_CheckedChanged);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(966, 616);
            this.Controls.Add(this.GUIUpdateChecked);
            this.Controls.Add(this.IIPTestCheckbox);
            this.Controls.Add(this.testTypeLabel);
            this.Controls.Add(this.aRateTestCheckBox);
            this.Controls.Add(this.gLoadTestCheckBox);
            this.Controls.Add(this.attitudeTestCheckBox);
            this.Controls.Add(this.GPSTestCheckBox);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.comboBox2);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.comboBox1);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.listBox1);
            this.Name = "Form1";
            this.Text = "RAMSES + POSNET Sniffer";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ListBox listBox1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.ComboBox comboBox1;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.ComboBox comboBox2;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.CheckBox GPSTestCheckBox;
        private System.Windows.Forms.CheckBox attitudeTestCheckBox;
        private System.Windows.Forms.CheckBox gLoadTestCheckBox;
        private System.Windows.Forms.CheckBox aRateTestCheckBox;
        private System.Windows.Forms.Label testTypeLabel;
        private System.Windows.Forms.CheckBox IIPTestCheckbox;
        private System.ComponentModel.BackgroundWorker backgroundWorker1;
        private System.Windows.Forms.CheckBox GUIUpdateChecked;
    }
}

