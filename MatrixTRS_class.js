/*
Static methods (8):
    -isMatrixTRS
    -translationMatrix
    -rotationMatrix
    -scaleMatrix
    -reflectionMatrix
    -projectionMatrix
    -fromLookAt
    -slerp

Instance methods (31):
    -isValid
    -copy
    -toString
    -toArray
    -toMat4
    -apply
    -equal
    -selectTransforms
    -getOrigin
    -getAngles
    -getScale
    -right
    -up
    -forward
    -determinant
    -eigenvalues
    -eigenvector    TODO: FIX!!!
    -setTransforms
    -setOrigin
    -setAngles
    -setScale
    -setViewVectors
    -lookAt
    -translate
    -rotate
    -scale
    -multiply
    -inverse
    -negate
    -reflect
    -project
*/

/**
 * This class facilitates the creation, manipulation, and conversion of 4x4 transformation matrices.
 * @author Gariam
 */
class MatrixTRS{
    /**
     * @param {(Vec3 | Mat4 | MatrixTRS | number[])=} origin - origin or 4x4 trasformation matrix
     * @param {Vec3=} angles - angles
     * @param {Vec3=} scale - scale
     */
    constructor(origin = Float64Array.of(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1), angles = new Vec3(0), scale = new Vec3(1)){
        if (!origin.hasOwnProperty("x")) this.m = new Float64Array(origin.m || origin);
        else {
            /**
             * The matrix as flat array of length 16.
             *
             * Do not change this unless you know what you are doing.
             * @type Float64Array
             */
            this.m = new Float64Array(16);
            origin = new Vec3(origin);

            //4th column (translation)
            this.m[12] = origin.x;
            this.m[13] = origin.y;
            this.m[14] = origin.z;
            this.m[15] = 1;

            const rotMatrix = MatrixTRS._anglesToMatrix(new Vec3(angles).multiply(0.0174533)); //convert angles to rotation matrix

            scale = new Vec3(scale);

            //1st column (scaled right vector)
            this.m[0] = rotMatrix[0] * scale.x;
            this.m[1] = rotMatrix[1] * scale.x;
            this.m[2] = rotMatrix[2] * scale.x;
            this.m[3] = 0;

            //2nd column (scaled up vector)
            this.m[4] = rotMatrix[4] * scale.y;
            this.m[5] = rotMatrix[5] * scale.y;
            this.m[6] = rotMatrix[6] * scale.y;
            this.m[7] = 0;

            //3rd column (scaled forward vector)
            this.m[8] = rotMatrix[8] * scale.z;
            this.m[9] = rotMatrix[9] * scale.z;
            this.m[10] = rotMatrix[10] * scale.z;
            this.m[11] = 0;
        }
        /**
         * When true the next called method that change the matrix will apply directly to this matrix instead returning a new one.
         * @type boolean
         */
        this._apply = false;
        /**
         * When true all methods that change the matrix will always apply directly to this matrix instead returning a new one.
         * @type boolean
         */
        this._persistentApply = false;
    }

    /**
     * Checks if the provided matrix is a valid transformation matrix.
     * @param {(MatrixTRS | Mat4 | number[])} matrix - matrix to check
     * @returns {boolean}
     */
    static isMatrixTRS(matrix){
        matrix = new Float32Array(matrix.m || matrix);
        if (matrix.length != 16) return false; //check if it's a 4x4 matrix
        if (matrix[3] != 0 || matrix[7] != 0 || matrix[11] != 0 || matrix[15] != 1) return false; //check if the last row is [0, 0, 0, 1]

        matrix = MatrixTRS.rotationMatrix(matrix).m; //remove scale component

        //repeated operations
        const a = matrix[5] * matrix[10] - matrix[6] * matrix[9];
        const m410 = matrix[4] * matrix[10];
        const m68 = matrix[6] * matrix[8];
        const b = matrix[4] * matrix[9] - matrix[5] * matrix[8];

        //If the determinant of the submatrix from 00 to 22 is negative or 0 then the matrix is not valid
        const determinant = matrix[0] * a - matrix[1] * (m410 - m68) + matrix[2] * b;
        if (determinant <= 0 || !determinant) return false;

        const transposed = Float32Array.of(matrix[0], matrix[4], matrix[8], matrix[1], matrix[5], matrix[9], matrix[2], matrix[6], matrix[10]);

        const adjugate = Float32Array.of(
            a,
            matrix[2] * matrix[9] - matrix[1] * matrix[10],
            matrix[1] * matrix[6] - matrix[2] * matrix[5],
            m68 - m410,
            matrix[0] * matrix[10] - matrix[2] * matrix[8],
            matrix[2] * matrix[4] - matrix[0] * matrix[6],
            b,
            matrix[1] * matrix[8] - matrix[0] * matrix[9],
            matrix[0] * matrix[5] - matrix[1] * matrix[4]);
        
        //if the inverse of the submatrix from 00 to 22 is different from the same submatrix transposed then it's not valid
        for (let i = 0; i < adjugate.length; i++){
            const inverse = adjugate[i] / determinant;
            if (Math.abs(inverse - transposed[i]) > 1e-6) return false;
        }
        return true;
    }

    /**
     * Returns a 4x4 transformation matrix with only the translation component.
     * @param {(Vec3 | MatrixTRS | Mat4 | number[])=} origin - translation vector or 4x4 transformation matrix
     * @returns {MatrixTRS} 4x4 translation matrix
     */
    static translationMatrix(origin = new Vec3(0)){
        if (!origin.hasOwnProperty("x")){
            origin = new Float64Array(origin.m || origin);
            origin = new Vec3(origin[12], origin[13], origin[14]);
        } else origin = new Vec3(origin);
        const matrix = new MatrixTRS();
        matrix.m[12] = origin.x;
        matrix.m[13] = origin.y;
        matrix.m[14] = origin.z;
        return matrix;
    }

    /**
     * Returns a 4x4 transformation matrix with only the rotation component.
     * @param {(Vec3 | MatrixTRS | Mat4 | number[])=} angles - angles vector or 4x4 transformation matrix
     * @returns {MatrixTRS} 4x4 rotation matrix
     */
    static rotationMatrix(angles = new Vec3(0)){
        const matrix = new MatrixTRS();
        if (!angles.hasOwnProperty("x")){
            matrix.m = new Float64Array(angles.m || angles);
            const currentScale = MatrixTRS._matrixToScale(matrix.m);
            matrix.m[0] /= currentScale.x;
            matrix.m[1] /= currentScale.x;
            matrix.m[2] /= currentScale.x;
            matrix.m[4] /= currentScale.y;
            matrix.m[5] /= currentScale.y;
            matrix.m[6] /= currentScale.y;
            matrix.m[8] /= currentScale.z;
            matrix.m[9] /= currentScale.z;
            matrix.m[10] /= currentScale.z;
            matrix.m[12] = 0;
            matrix.m[13] = 0;
            matrix.m[14] = 0;
        } else matrix.m = MatrixTRS._anglesToMatrix(new Vec3(angles).multiply(0.0174533));
        return matrix;
    }

    /**
     * Returns a 4x4 transformation matrix with only the scale component.
     * @param {(Vec3 | MatrixTRS | Mat4 | number[])=} scale - scale vector or 4x4 transformation matrix
     * @returns {MatrixTRS} 4x4 scale matrix
     */
    static scaleMatrix(scale = new Vec3(1)){
        if (!scale.hasOwnProperty("x")){
            scale = MatrixTRS._matrixToScale(scale.m || scale);
        } else scale = new Vec3(scale);
        const matrix = new MatrixTRS();
        matrix.m[0] = scale.x;
        matrix.m[5] = scale.y;
        matrix.m[10] = scale.z;
        return matrix;
    }

    /**
     * Returns a 4x4 reflection matrix with the given normal vector.
     * @param {Vec3} normal - normal vector
     * @param {Vec3=} origin - origin of the normal vector
     * @returns {Mat4} reflection matrix
     */
    static reflectionMatrix(normal, origin = new Vec3(0)){
        //repeated operations
        const xy = normal.x * normal.y;
        const xz = normal.x * normal.z;
        const yz = normal.y * normal.z;

        //I - 2 * normal * transpose(normal)
        const matrix = Float64Array.of(normal.x * normal.x, xy, xz, 0,
                                       xy, normal.y * normal.y, yz, 0,
                                       xz, yz, normal.z * normal.z, 0,
                                       origin.x, origin.y, origin.z, 1);
        
        matrix[0] += matrix[0] - 1;
        matrix[1] += matrix[1];
        matrix[2] += matrix[2];
        matrix[4] += matrix[4];
        matrix[5] += matrix[5] - 1;
        matrix[6] += matrix[6];
        matrix[8] += matrix[8];
        matrix[9] += matrix[9];
        matrix[10] += matrix[10] - 1;
        matrix[12] *= 1 - normal.x;
        matrix[13] *= 1 - normal.y;
        matrix[14] *= 1 - normal.z;
        
        const mat4 = new Mat4();
        mat4.m = Array.from(matrix);
        return mat4;
    }

    /**
     * Returns a 4x4 projection matrix with the given normal vector.
     * @param {Vec3} normal - normal vector
     * @param {Vec3=} origin - origin of the normal vector
     * @returns {Mat4} projection matrix
     */
    static projectionMatrix(normal, origin = new Vec3(0)){
        //repeated operations
        const xy = normal.x * normal.y;
        const xz = normal.x * normal.z;
        const yz = normal.y * normal.z;

        //I - normal * transpose(normal)
        const matrix = new Mat4();
        matrix.m = [1 - normal.x * normal.x, -xy, -xz, 0,
                    -xy, 1 - normal.y * normal.y, -yz, 0,
                    -xz, -yz, 1 - normal.z * normal.z, 0,
                    origin.x, origin.y, origin.z, 1];

        return matrix;
    }

    /**
     * Returns a 4x4 transformation matrix from the camera components.
     * @param {Vec3} eye - position of the camera
     * @param {Vec3} center - position the camera is looking at
     * @param {Vec3} up - up vector
     * @returns {MatrixTRS} projection matrix
     */
    static fromLookAt(eye, center, up){
        const forward = center.subtract(eye).normalize();
        const right = up.cross(forward).normalize();
        const newUp = right.cross(forward);

        return new MatrixTRS(Float64Array.of(
            right.x, right.y, right.z, 0,
            newUp.x, newUp.y, newUp.z, 0,
            forward.x, forward.y, forward.z, 0,
            eye.x, eye.y, eye.z, 1));
    }

    /**
     * Spherical linear interpolation between two transformation matrices.
     * @param {(MatrixTRS | Mat4 | number[])} matrixA - 4x4 transformation matrix
     * @param {(MatrixTRS | Mat4 | number[])} matrixB - 4x4 transformation matrix
     * @param {number} value - interpolation factor
     * @param {number=} valueRot - rotation interpolation factor
     * @param {number=} valueScl - scale interpolation factor
     * @returns {MatrixTRS}
     */
    static slerp(matrixA, matrixB, value, valueRot, valueScl){
        valueRot = valueRot || value;
        valueScl = valueScl || value;
        if (value == 0 && valueRot == 0 && valueScl == 0) return new MatrixTRS(matrixA);
        else if (value == 1 && valueRot == 1 && valueScl == 1) return new MatrixTRS(matrixB);

        matrixA = new Float64Array(matrixA.m || matrixA);
        matrixB = new Float64Array(matrixB.m || matrixB);

        function mixVec3(vectorA, vectorB, value){
            vectorA.x = vectorA.x * (1 - value) + vectorB.x * value;
            vectorA.y = vectorA.y * (1 - value) + vectorB.y * value;
            vectorA.z = vectorA.z * (1 - value) + vectorB.z * value;
            return vectorA;
        }

        let scale, scaleA, scaleB;
        switch (valueScl){
            case 0: scale = scaleA = MatrixTRS._matrixToScale(matrixA); break;
            case 1: scale = scaleB = MatrixTRS._matrixToScale(matrixB); break;
            default:
                scaleA = MatrixTRS._matrixToScale(matrixA);
                scaleB = MatrixTRS._matrixToScale(matrixB);
                scale = mixVec3(scaleA, scaleB, valueScl);
        }
        
        let matrix;
        switch (valueRot) {
            case 0: matrix = new Float64Array(matrixA[0] / scaleA.x, matrixA[1] / scaleA.x, matrixA[2] / scaleA.x, 0, matrixA[4] / scaleA.y, matrixA[5] / scaleA.y, matrixA[6] / scaleA.y, 0, matrixA[8] / scaleA.z, matrixA[9] / scaleA.z, matrixA[10] / scaleA.z, 0, 0, 0, 0, 1); break;
            case 1: matrix = new Float64Array(matrixB[0] / scaleB.x, matrixB[1] / scaleB.x, matrixB[2] / scaleB.x, 0, matrixB[4] / scaleB.y, matrixB[5] / scaleB.y, matrixB[6] / scaleB.y, 0, matrixB[8] / scaleB.z, matrixB[9] / scaleB.z, matrixB[10] / scaleB.z, 0, 0, 0, 0, 1); break;
            default:
                const q1 = MatrixTRS._matrixToQuaternion(matrixA);
                const q2 = MatrixTRS._matrixToQuaternion(matrixB);
                const quaternion = MatrixTRS._slerpQuaternion(q1, q2, valueRot);
                matrix = MatrixTRS._quaternionToMatrix(quaternion);
        }

        let origin;
        switch (value) {
            case 0: origin = new Vec3(matrixA[12], matrixA[13], matrixA[14]);
            case 1: origin = new Vec3(matrixB[12], matrixB[13], matrixB[14]);
            default: origin = mixVec3(new Vec3(matrixA[12], matrixA[13], matrixA[14]), new Vec3(matrixB[12], matrixB[13], matrixB[14]), value);
        }

        matrix[0] *= scale.x;
        matrix[1] *= scale.x;
        matrix[2] *= scale.x;
        matrix[4] *= scale.y;
        matrix[5] *= scale.y;
        matrix[6] *= scale.y;
        matrix[8] *= scale.z;
        matrix[9] *= scale.z;
        matrix[10] *= scale.z;
        matrix[12] = origin.x;
        matrix[13] = origin.y;
        matrix[14] = origin.z;

        return new MatrixTRS(matrix);
    }

    /**
     * Checks if it's still a valid transformation matrix.
     * @returns {boolean}
     */
    isValid(){
        this._apply = false;
        return MatrixTRS.isMatrixTRS(this.m);
    }

    /**
     * Returns an identical transformation matrix.
     * @returns {MatrixTRS}
     */
    copy(){
        this._apply = false;
        return new MatrixTRS(this);
    }

    /**
     * Returns the matrix as a string.
     * @returns {string}
     */
    toString(){
        this._apply = false;
		return this.m[0]+' '+this.m[1]+' '+this.m[2]+' '+this.m[3]+' '+
			    this.m[4]+' '+this.m[5]+' '+this.m[6]+' '+this.m[7]+' '+
				this.m[8]+' '+this.m[9]+' '+this.m[10]+' '+this.m[11]+' '+
				this.m[12]+' '+this.m[13]+' '+this.m[14]+' '+this.m[15];
	}

    /**
     * Returns the matrix as an array.
     * @returns {string}
     */
    toArray(){
        this._apply = false;
        return Array.from(this.m);
    }

    /**
     * Returns the matrix as a Mat4 object.
     * @returns {Mat4}
     */
    toMat4(){
        this._apply = false;
        const mat4 = new Mat4();
        mat4.m = Array.from(this.m);
        return mat4;
    }

    /**
     * Use this before any method that changes the matrix to apply the result directly to this matrix instead of returning it.
     * @param {boolean} persistent if to always have this behaviour, without the need to call this function again
     * @returns {MatrixTRS} this matrix 
     */
    apply(persistent = false){
        this._apply = !persistent;
        this._persistentApply = persistent;
        return this;
    }

    /**
     * Check if this matrix is equal to the provided matrix
     * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 matrix
     * @returns 
     */
    equal(matrix) {
        matrix = new Float64Array(matrix.m || matrix);
        for (let i = 0; i < this.m.length; i++) {
            if (Math.abs(this.m[i] - matrix[i]) > 1e-6) return false;
        }
        return true;
    }

    /**
     * Extracts a transformation matrix with only the specified components and returns it.
     * @param {boolean} origin
     * @param {boolean} angles
     * @param {boolean} scale
     * @returns {MatrixTRS}
     */
    selectTransforms(origin, angles, scale){
        this._apply = false;
        const newMatrix = this.copy();
        let newScale;

        if (!origin){
            newMatrix.m[12] = 0;
            newMatrix.m[13] = 0;
            newMatrix.m[14] = 0;
        }
        if (!angles){
            if (scale) newScale = MatrixTRS._matrixToScale(this.m);
            else newScale = new Vec3(1);
            newMatrix.m[0] = newScale.x;
            newMatrix.m[1] = 0;
            newMatrix.m[2] = 0;
            newMatrix.m[4] = 0;
            newMatrix.m[5] = newScale.y;
            newMatrix.m[6] = 0;
            newMatrix.m[8] = 0;
            newMatrix.m[9] = 0;
            newMatrix.m[10] = newScale.z;
        } else if (!scale) {
            const currentScale = MatrixTRS._matrixToScale(this.m);
            newMatrix.m[0] /= currentScale.x;
            newMatrix.m[1] /= currentScale.x;
            newMatrix.m[2] /= currentScale.x;
            newMatrix.m[4] /= currentScale.y;
            newMatrix.m[5] /= currentScale.y;
            newMatrix.m[6] /= currentScale.y;
            newMatrix.m[8] /= currentScale.z;
            newMatrix.m[9] /= currentScale.z;
            newMatrix.m[10] /= currentScale.z;
        }
        return newMatrix;
    }

    /**
     * Returns the origin component.
     * @returns {Vec3} origin
     */
    getOrigin(){
        this._apply = false;
        return new Vec3(this.m[12], this.m[13], this.m[14]);
    }

    /**
     * Returns the angles component.
     * @returns {Vec3} angles
     */
    getAngles(){
        this._apply = false;
        return MatrixTRS._matrixToAngles(this.m).multiply(57.2958);
    }

    /**
     * Returns the scale component.
     * @returns {Vec3} scale
     */
    getScale(){
        this._apply = false;
        return MatrixTRS._matrixToScale(this.m);
    }

    /**
     * Returns the right view vector normalized.
     * @returns {Vec3} normalized right vector
     */
    right(){
        this._apply = false;
        return new Vec3(this.m[0], this.m[1], this.m[2]).normalize();
    }

    /**
     * Returns the up view vector normalized.
     * @returns {Vec3} normalized up vector
     */
    up(){
        this._apply = false;
        return new Vec3(this.m[4], this.m[5], this.m[6]).normalize();
    }

    /**
     * Returns the forward view vector normalized.
     * @returns {Vec3} normalized forward vector
     */
    forward(){
        this._apply = false;
        return new Vec3(this.m[8], this.m[9], this.m[10]).normalize();
    }

    /**
     * Returns the determinant of the matrix.
     * @returns {number} determinant
     */
    determinant(){
        this._apply = false;
        return this.m[0] * (this.m[5] * this.m[10] - this.m[6] * this.m[9]) -
                this.m[1] * (this.m[4] * this.m[10] - this.m[6] * this.m[8]) +
                this.m[2] * (this.m[4] * this.m[9] - this.m[5] * this.m[8]);
    }

    /**
     * Returns the 3 eigenvalues of the matrix, NaN when they are complex numbers.
     * 
     * They are only a close approximation and the accuracy is dependent on the javascript framework itself because of the use of the Math.cbrt() function.
     * @returns {number[]} eigenvalues
     */
    eigenvalues(){
        this._apply = false;

        //calculate the coefficients of the characteristic polynomial
        const c0 = this.determinant();
        const c1 = -(this.m[0] * this.m[5] + this.m[0] * this.m[10] + this.m[5] * this.m[10] - this.m[1] * this.m[4] - this.m[2] * this.m[8] - this.m[6] * this.m[9]);
        const c2 = this.m[0] + this.m[5] + this.m[10];
        
        //solve the cubic equation
        const commonFactor = c2 / -3;
        const q = (-3 * c1 - c2 * c2) / 9;
        const r = (-9 * c2 * c1 - 27 * c0 - 2 * c1 * c1 * c1) / -54;
        const q3 = q * q * q;
        const disc = q3 + r * r;

        //find the roots (eigenvalues)
        let lambda1, lambda2, lambda3;
        if (disc == 0){
            const rRT = Math.cbrt(r);
            lambda1 = rRT + rRT - commonFactor;
            lambda2 = lambda3 = -(rRT + rRT) / 2 - commonFactor;
        } else if (disc > 0){
            const discRT = Math.sqrt(disc);
            const s = Math.cbrt(r + discRT);
            const t = Math.cbrt(r - discRT);
            lambda1 = s + t - commonFactor;
            lambda2 = lambda3 = NaN; //complex numbers: -(s + t) / 2 - commonFactor ± i√3 / 2 * (s - t)
        } else {
            const theta = Math.acos(r / Math.sqrt(-q3));
            const pi2 = Math.PI + Math.PI;
            const qRT = Math.sqrt(-q) * 2;
            lambda1 = qRT * Math.cos(theta / 3) - commonFactor;
            lambda2 = qRT * Math.cos((theta + pi2) / 3) - commonFactor;
            lambda3 = qRT * Math.cos((theta + pi2 + pi2) / 3) - commonFactor;
        }
        
        return [lambda1, lambda2, lambda3];
    }

    /**
     * TODO: FIX!!!
     * Returns the eigenvector corresponding to the provided eigenvalue.
     * 
     * If eigenvalue is NaN, null is returned, while if it's not provided, it defaults to 1.
     * @param {number=} eigenvalue - eigenvalue
     * @returns {(Vec3 | null)} eigenvector
     */
    eigenvector(eigenvalue = 1){
        this._apply = false;
        if (Number.isNaN(eigenvalue)) return null;

        //calculate the matrix A - λI
        const a = this.m[0] - eigenvalue;
        const e = this.m[5] - eigenvalue;
        const i = this.m[10] - eigenvalue;

        //calculate the nullspace of the matrix A - λI
        const vector = new Vec3();
        if (Math.abs(a) > Math.abs(this.m[4])) {
            if (Math.abs(a) > Math.abs(this.m[8])) {
                vector.y = 1;
                vector.z = 0;
                vector.x = -(this.m[1] * vector.y + this.m[2] * vector.z) / a;
            } else {
                vector.x = 1;
                vector.y = 0;
                vector.z = -(this.m[8] * vector.x + this.m[9] * vector.y) / i;
            }
        } else {
            if (Math.abs(this.m[4]) > Math.abs(this.m[8])) {
                vector.y = 1;
                vector.z = 0;
                vector.x = -(this.m[4] * vector.y + this.m[6] * vector.z) / e;
            } else {
                vector.x = 1;
                vector.y = 0;
                vector.z = -(this.m[8] * vector.x + this.m[9] * vector.y) / i;
            }
        }
        return vector;
    }

    /**
     * Sets new transforms for the matrix.
     * @param {(Vec3 | MatrixTRS | Mat4 | number[])=} origin - origin or 4x4 transformation matrix
     * @param {Vec3=} angles - angles
     * @param {Vec3=} scale - scale
     */
    setTransforms(origin, angles, scale){
        let matrix;
        if (this._apply || this._persistentApply) matrix = this.m;

        if (!origin.hasOwnProperty("x")) matrix = new Float64Array(origin.m || origin);
        else {
            origin = origin || new Vec3(matrix[12], matrix[13], matrix[14]);

            //4th column (translation)
            matrix[12] = origin.x;
            matrix[13] = origin.y;
            matrix[14] = origin.z;

            let rotMatrix;
            if (angles) rotMatrix = MatrixTRS._anglesToMatrix(new Vec3(angles).multiply(0.0174533)); //convert angles to rotation matrix
            else rotMatrix = MatrixTRS.rotationMatrix(matrix).m;

            scale = scale || this.getScale();

            //1st column (scaled right vector)
            matrix[0] = rotMatrix[0] * scale.x;
            matrix[1] = rotMatrix[1] * scale.x;
            matrix[2] = rotMatrix[2] * scale.x;

            //2nd column (scaled up vector)
            matrix[4] = rotMatrix[4] * scale.y;
            matrix[5] = rotMatrix[5] * scale.y;
            matrix[6] = rotMatrix[6] * scale.y;

            //3rd column (scaled forward vector)
            matrix[8] = rotMatrix[8] * scale.z;
            matrix[9] = rotMatrix[9] * scale.z;
            matrix[10] = rotMatrix[10] * scale.z;
        }

        if (this._apply) this._apply = false;
        else return new MatrixTRS(matrix);
    }

    /**
     * Changes the origin component and returns the resulting matrix.
     * @param {Vec3} origin - origin
     */
    setOrigin(origin){
        let matrix;
        if (this._apply || this._persistentApply) matrix = this.m;
        else matrix = this.m.slice();

        matrix[12] = origin.x;
        matrix[13] = origin.y;
        matrix[14] = origin.z;

        if (this._apply) this._apply = false;
        else return new MatrixTRS(matrix);
    }

    /**
     * Changes the angles component and returns the resulting matrix.
     * @param {Vec3} angles - angles
     */
    setAngles(angles){
        let matrix;
        if (this._apply || this._persistentApply) matrix = this.m;
        else matrix = this.m.slice();

        const magnitude = MatrixTRS._matrixToScale(matrix);
        const rotMatrix = MatrixTRS._anglesToMatrix(new Vec3(angles).multiply(0.0174533));

        //right vector
        matrix[0] = rotMatrix[0] * magnitude.x;
        matrix[1] = rotMatrix[1] * magnitude.x;
        matrix[2] = rotMatrix[2] * magnitude.x;

        //up vector
        matrix[4] = rotMatrix[4] * magnitude.y;
        matrix[5] = rotMatrix[5] * magnitude.y;
        matrix[6] = rotMatrix[6] * magnitude.y;

        //forward vector
        matrix[8] = rotMatrix[8] * magnitude.z;
        matrix[9] = rotMatrix[9] * magnitude.z;
        matrix[10] = rotMatrix[10] * magnitude.z;

        if (this._apply) this._apply = false;
        else return new MatrixTRS(matrix);
    }

    /**
     * Changes the scale component and returns the resulting matrix.
     * @param {Vec3} scale - scale
     */
    setScale(scale){
        let matrix;
        if (this._apply || this._persistentApply) matrix = this.m;
        else matrix = this.m.slice();

        const right = this.getRight();
        const up = this.getUp();
        const forward = right.cross(up); //avoids 1 square root

        matrix[0] = right.x * scale.x;
        matrix[1] = right.y * scale.x;
        matrix[2] = right.z * scale.x;
        matrix[4] = up.x * scale.y;
        matrix[5] = up.y * scale.y;
        matrix[6] = up.z * scale.y;
        matrix[8] = forward.x * scale.z;
        matrix[9] = forward.y * scale.z;
        matrix[10] = forward.z * scale.z;

        if (this._apply) this._apply = false;
        else return new MatrixTRS(matrix);
    }

    /**
     * Changes the view vectors and returns the resulting matrix.
     * @param {Vec3} right - normalized right vector
     * @param {Vec3} up - normalized up vector
     * @param {Vec3} forward - normalized forward vector
     */
    setViewVectors(right, up, forward){
        let matrix;
        if (this._apply || this._persistentApply) matrix = this.m;
        else matrix = this.m.slice();

        const scale = MatrixTRS._matrixToScale(matrix);
        right = right.multiply(scale.x);
        up = up.multiply(scale.y);
        forward = forward.multiply(scale.z);

        matrix[0] = right.x;
        matrix[1] = right.y;
        matrix[2] = right.z;
        matrix[4] = up.x;
        matrix[5] = up.y;
        matrix[6] = up.z;
        matrix[8] = forward.x;
        matrix[9] = forward.y;
        matrix[10] = forward.z;

        if (this._apply) this._apply = false;
        else return new MatrixTRS(matrix);
    }

    /**
     * Rotates the matrix to make the forward vector point at a specific point in space and returns the resulting matrix.
     * @param {Vec3} point - point to look at
     * @param {Vec3} upVector - up vector
     * @returns {MatrixTRS} rotated matrix
     */
    lookAt(point, upVector){
        let matrix;
        if (this._apply || this._persistentApply) matrix = this.m;
        else matrix = this.m.slice();

        const forward = this.getOrigin().subtract(point).normalize();
        const right = upVector.cross(forward).normalize();
        const up = forward.cross(right);

        matrix[0] = right.x;
        matrix[1] = right.y;
        matrix[2] = right.z;
        matrix[4] = up.x;
        matrix[5] = up.y;
        matrix[6] = up.z;
        matrix[8] = forward.x;
        matrix[9] = forward.y;
        matrix[10] = forward.z;
        
        if (this._apply) this._apply = false;
        else return new MatrixTRS(matrix);
    }

    /**
     * Applies a translation and returns the resulting matrix.
     * @param {Vec3} translation - translation
     * @returns {MatrixTRS} translated matrix
     */
    translate(translation){
        let matrix;
        if (this._apply || this._persistentApply) matrix = this.m;
        else matrix = this.m.slice();

        matrix[12] += translation.x;
        matrix[13] += translation.y;
        matrix[14] += translation.z;

        if (this._apply) this._apply = false;
        else return new MatrixTRS(matrix);
    }

    /**
     * Applies a rotation and returns the resulting matrix.
     * @param {Vec3} angles - angles
     * @returns {MatrixTRS} rotated matrix
     */
    rotate(angles){
        const rotMatrix = MatrixTRS._anglesToMatrix(new Vec3(angles).multiply(0.0174533)); //convert angles to rotation matrix
        const newMatrix = this.multiply(rotMatrix); //multiplies to rotation matrix with the current transformation matrix

        if (this._apply || this._persistentApply) {
            this._apply = false;
            this.m = newMatrix.m;
        } else return newMatrix;
    }

    /**
     * Applies scaling and returns the resulting matrix.
     * @param {Vec3} scale - scale
     * @returns {MatrixTRS} scaled matrix
     */
    scale(scale){
        let matrix;
        if (this._apply || this._persistentApply) matrix = this.m;
        else matrix = this.m.slice();

        matrix[0] *= scale.x;
        matrix[1] *= scale.x;
        matrix[2] *= scale.x;
        matrix[4] *= scale.y;
        matrix[5] *= scale.y;
        matrix[6] *= scale.y;
        matrix[8] *= scale.z;
        matrix[9] *= scale.z;
        matrix[10] *= scale.z;

        if (this._apply) this._apply = false;
        else return new MatrixTRS(matrix);
    }

    /**
     * Multiplies the matrix and returns the result.
     * @param {(MatrixTRS | Mat4 | Vec3 | Vec2 | number[])} multiplier - 4x4 transformation matrix or a vector
     * @returns {(MatrixTRS | Vec3)} 4x4 transformation matrix or a vector
     */
    multiply(multiplier){
        let newMatrix;

        function vectorMultiply(matrix, vector){
            if (!(vector instanceof Vec3)) vector = new Vec3(vector);
            
            const x = vector.x * matrix[0] + vector.y * matrix[4] + vector.z * matrix[8] + matrix[12];
            const y = vector.x * matrix[1] + vector.y * matrix[5] + vector.z * matrix[9] + matrix[13];
            const z = vector.x * matrix[2] + vector.y * matrix[6] + vector.z * matrix[10] + matrix[14];
        
            return new Vec3(x, y, z);
        }

        if (multiplier.hasOwnProperty("x")) return vectorMultiply(this.m, multiplier);
        else {
            matrix = new Float64Array(multiplier.m || multiplier);

            const row0 = new Vec3(matrix[0], matrix[1], matrix[2]);
            const row1 = new Vec3(matrix[4], matrix[5], matrix[6]);
            const row2 = new Vec3(matrix[8], matrix[9], matrix[10]);

            const column0 = vectorMultiply(this.m, row0);
            const column1 = vectorMultiply(this.m, row1);
            const column2 = vectorMultiply(this.m, row2);
            const column3 = new Vec3(this.m[12] + matrix[12], this.m[13] + matrix[13], this.m[14] + matrix[14]);
        
            newMatrix = Float64Array.of(
                column0.x, column0.y, column0.z, 0,
                column1.x, column1.y, column1.z, 0,
                column2.x, column2.y, column2.z, 0,
                column3.x, column3.y, column3.z, 1);
        }

        if (this._apply || this._persistentApply) {
            this._apply = false;
            this.m = newMatrix;
        } else return new MatrixTRS(newMatrix);
    }

    /**
     * Returns the inverse of the matrix.
     * @returns {MatrixTRS} inverted matrix
     */
    inverse(){
        let matrix;
        if (this._apply || this._persistentApply) matrix = this.m;
        else matrix = this.m.slice();

        let right = new Vec3(this.m[0], this.m[1], this.m[2]);
        let up = new Vec3(this.m[4], this.m[5], this.m[6]);
        let forward = new Vec3(this.m[8], this.m[9], this.m[10]);

        //Inverse scale
        right = right.divide(right.lengthSqr());
        up = up.divide(up.lengthSqr());
        forward = forward.divide(forward.lengthSqr());

        //Inverse rotation
        matrix[0] = right.x;
        matrix[1] = up.x;
        matrix[2] = forward.x;
        matrix[4] = right.y;
        matrix[5] = up.y;
        matrix[6] = forward.y;
        matrix[8] = right.z;
        matrix[9] = up.z;
        matrix[10] = forward.z;

        //Inverse translation
        matrix[12] = -matrix[12];
        matrix[13] = -matrix[13];
        matrix[14] = -matrix[14];

        if (this._apply) this._apply = false;
        else return new MatrixTRS(matrix);
    }

    /**
     * Negates every component of the matrix and returns the result.
     * @returns {MatrixTRS} negated matrix
     */
    negate(){
        const matrix = Float64Array.from(this.m, value => -value);
        matrix[15] = 1

        if (this._apply || this._persistentApply) {
            this._apply = false;
            this.m = matrix;
        } else return new MatrixTRS(matrix);
    }

    /**
     * Reflects the matrix on the plane described by the given normal vector and returns the result.
     * @param {Vec3} normal - normal vector
     * @param {Vec3=} origin - origin of the normal vector
     * @returns {MatrixTRS} reflected matrix
     */
    reflect(normal, origin = new Vec3(0)){
        const reflectionMatrix = MatrixTRS.reflectionMatrix(normal, origin);
        const newMatrix = this.multiply(reflectionMatrix);

        if (this._apply || this._persistentApply) {
            this._apply = false;
            this.m = matrix.m;
        } else return newMatrix;
    }

    /**
     * Projects the matrix on the plane described by the given normal vector and returns the result.
     * @param {Vec3} normal - normal vector
     * @param {Vec3=} origin - origin of the normal vector
     * @returns {MatrixTRS} projected matrix
     */
    project(normal, origin = new Vec3(0)){
        const projectionMatrix = MatrixTRS.projectionMatrix(normal, origin);
        const newMatrix = this.multiply(projectionMatrix);

        if (this._apply || this._persistentApply) {
            this._apply = false;
            this.m = matrix.m;
        } else return newMatrix;
    }

    static _anglesToMatrix(angles){
        const sinAlpha = Math.sin(angles.x), cosAlpha = Math.cos(angles.x);
        const sinBeta = Math.sin(angles.y), cosBeta = Math.cos(angles.y);
        const sinGamma = Math.sin(angles.z), cosGamma = Math.cos(angles.z);

        //repeated operations
        const sinAlpha_sinBeta = sinAlpha * sinBeta;
        const cosAlpha_sinBeta = cosAlpha * sinBeta;

        const m11 = cosGamma * cosBeta;
        const m12 = sinGamma * cosBeta;
        const m13 = -sinBeta;
        const m21 = cosGamma * sinAlpha_sinBeta - sinGamma * cosAlpha;
        const m22 = sinGamma * sinAlpha_sinBeta + cosGamma * cosAlpha;
        const m23 = cosBeta * sinAlpha;
        const m31 = cosGamma * cosAlpha_sinBeta + sinGamma * sinAlpha;
        const m32 = sinGamma * cosAlpha_sinBeta - cosGamma * sinAlpha;
        const m33 = cosBeta * cosAlpha;

        return Float64Array.of(m11, m12, m13, 0, m21, m22, m23, 0, m31, m32, m33, 0, 0, 0, 0, 1);
    }

    static _matrixToAngles(matrix){
        const right = new Vec3(matrix[0], matrix[1], matrix[2]).normalize();
        const up = new Vec3(matrix[4], matrix[5], matrix[6]).normalize();
        const forward = right.cross(up); //avoids 1 square root
        const angles = new Vec3();
        
        if (1.0 - Math.abs(right.z) > 1e-6){
            angles.x = Math.atan2(up.z, forward.z);
            angles.y = -Math.asin(right.z);
            angles.z = Math.atan2(right.y, right.x);
        } else {
            if (right.z > 0){
                angles.x = 1.5707963268;
                angles.y = Math.atan2(right.x, up.x);
                angles.z = 0;
            } else {
                angles.x = -1.5707963268;
                angles.y = -Math.atan2(right.x, up.x);
                angles.z = 0;
            }
        }
        return angles;
    }

    static _matrixToScale(matrix){
        const x = new Vec3(matrix[0], matrix[1], matrix[2]).length();
        const y = new Vec3(matrix[4], matrix[5], matrix[6]).length();
        const z = new Vec3(matrix[8], matrix[9], matrix[10]).length();
        return new Vec3(x, y, z);
    }

    static _matrixToQuaternion(matrix) {
        const trace = matrix[0] + matrix[5] + matrix[10];
        let w, x, y, z;

        if (trace > 0) {
            const s = 0.5 / Math.sqrt(trace + 1);
            w = 0.25 / s;
            x = (matrix[9] - matrix[6]) * s;
            y = (matrix[2] - matrix[8]) * s;
            z = (matrix[4] - matrix[1]) * s;
        } else if (matrix[0] > matrix[5] && matrix[0] > matrix[10]) {
            const s = 2 * Math.sqrt(1 + matrix[0] - matrix[5] - matrix[10]);
            w = (matrix[9] - matrix[6]) / s;
            x = 0.25 * s;
            y = (matrix[1] + matrix[4]) / s;
            z = (matrix[2] + matrix[8]) / s;
        } else if (matrix[5] > matrix[10]) {
            const s = 2 * Math.sqrt(1 + matrix[5] - matrix[0] - matrix[10]);
            w = (matrix[2] - matrix[8]) / s;
            x = (matrix[1] + matrix[4]) / s;
            y = 0.25 * s;
            z = (matrix[6] + matrix[9]) / s;
        } else {
            const s = 2 * Math.sqrt(1 + matrix[10] - matrix[0] - matrix[5]);
            w = (matrix[4] - matrix[1]) / s;
            x = (matrix[2] + matrix[8]) / s;
            y = (matrix[6] + matrix[9]) / s;
            z = 0.25 * s;
        }

        const length = Math.sqrt(w * w + x * x + y * y + z * z);
        return Float64Array.of(w / length, x / length, y / length, z / length);
    }

    static _quaternionToMatrix(q) {
        const w = q[0], x = q[1], y = q[2], z = q[3];
        const xx = x * x, xy = x * y, xz = x * z, xw = x * w;
        const yy = y * y, yz = y * z, yw = y * w;
        const zz = z * z, zw = z * w;
        return Float64Array.of(
            1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw), 0,
            2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw), 0,
            2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy), 0,
            0, 0, 0, 1);
    }

    static _slerpQuaternion(q1, q2, t) {
        let dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

        if (dot < 0) {
            q2 = Float64Array.from(q2, value => -value);
            dot = -dot;
        }

        if (dot > 0.9995) {
            return Float64Array.of(q1[0] * (1 - t) + q2[0] * t,
                    q1[1] * (1 - t) + q2[1] * t,
                    q1[2] * (1 - t) + q2[2] * t,
                    q1[3] * (1 - t) + q2[3] * t);
        }

        const theta = Math.acos(dot);
        const sinTheta = Math.sin(theta);
        const w1 = Math.sin((1 - t) * theta) / sinTheta;
        const w2 = Math.sin(t * theta) / sinTheta;

        return Float64Array.of(
                q1[0] * w1 + q2[0] * w2,
                q1[1] * w1 + q2[1] * w2,
                q1[2] * w1 + q2[2] * w2,
                q1[3] * w1 + q2[3] * w2);
    }
}