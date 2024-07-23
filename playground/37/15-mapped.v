// Benchmark "adder" written by ABC on Thu Jul 18 07:01:54 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n180, new_n181,
    new_n182, new_n183, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n228, new_n229, new_n230,
    new_n231, new_n232, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n246,
    new_n247, new_n248, new_n249, new_n250, new_n251, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n303,
    new_n306, new_n308, new_n310;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  tech160nm_fioai012aa1n03p5x5 g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  nor022aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand42aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n16x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n06x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  oai012aa1n02x5               g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  norp02aa1n04x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor022aa1n06x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  tech160nm_fixnrc02aa1n03p5x5 g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nor043aa1n06x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n109), .b(new_n117), .o1(new_n118));
  orn002aa1n03x5               g023(.a(\a[5] ), .b(\b[4] ), .o(new_n119));
  tech160nm_fioaoi03aa1n05x5   g024(.a(\a[6] ), .b(\b[5] ), .c(new_n119), .o1(new_n120));
  tech160nm_fiaoi012aa1n03p5x5 g025(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n121));
  oaib12aa1n09x5               g026(.a(new_n121), .b(new_n114), .c(new_n120), .out0(new_n122));
  nanb02aa1n02x5               g027(.a(new_n122), .b(new_n118), .out0(new_n123));
  xnrc02aa1n12x5               g028(.a(\b[8] ), .b(\a[9] ), .out0(new_n124));
  inv000aa1d42x5               g029(.a(new_n124), .o1(new_n125));
  aoi012aa1n02x5               g030(.a(new_n98), .b(new_n123), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  nor022aa1n08x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nand02aa1n06x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  inv000aa1d42x5               g035(.a(\b[9] ), .o1(new_n131));
  oaoi03aa1n09x5               g036(.a(new_n97), .b(new_n131), .c(new_n98), .o1(new_n132));
  xnrc02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .out0(new_n133));
  nor042aa1n02x5               g038(.a(new_n133), .b(new_n124), .o1(new_n134));
  aoai13aa1n03x5               g039(.a(new_n134), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n130), .b(new_n135), .c(new_n132), .out0(\s[11] ));
  aob012aa1n02x5               g041(.a(new_n130), .b(new_n135), .c(new_n132), .out0(new_n137));
  tech160nm_fioai012aa1n03p5x5 g042(.a(new_n137), .b(\b[10] ), .c(\a[11] ), .o1(new_n138));
  xorb03aa1n02x5               g043(.a(new_n138), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor022aa1n08x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand02aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nona23aa1d18x5               g046(.a(new_n141), .b(new_n129), .c(new_n128), .d(new_n140), .out0(new_n142));
  norp03aa1n02x5               g047(.a(new_n142), .b(new_n133), .c(new_n124), .o1(new_n143));
  aoai13aa1n06x5               g048(.a(new_n143), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n144));
  aoi012aa1n06x5               g049(.a(new_n140), .b(new_n128), .c(new_n141), .o1(new_n145));
  oai012aa1n18x5               g050(.a(new_n145), .b(new_n142), .c(new_n132), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  nor042aa1n04x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  nand42aa1d28x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  norb02aa1n02x5               g054(.a(new_n149), .b(new_n148), .out0(new_n150));
  xnbna2aa1n03x5               g055(.a(new_n150), .b(new_n144), .c(new_n147), .out0(\s[13] ));
  inv000aa1d42x5               g056(.a(\a[13] ), .o1(new_n152));
  inv000aa1d42x5               g057(.a(\b[12] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n144), .b(new_n147), .o1(new_n154));
  oaoi03aa1n02x5               g059(.a(new_n152), .b(new_n153), .c(new_n154), .o1(new_n155));
  xnrb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n09x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nanp02aa1n24x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nano23aa1d15x5               g063(.a(new_n148), .b(new_n157), .c(new_n158), .d(new_n149), .out0(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n06x5               g065(.a(new_n158), .b(new_n157), .c(new_n152), .d(new_n153), .o1(new_n161));
  aoai13aa1n04x5               g066(.a(new_n161), .b(new_n160), .c(new_n144), .d(new_n147), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  xorc02aa1n12x5               g069(.a(\a[15] ), .b(\b[14] ), .out0(new_n165));
  xorc02aa1n12x5               g070(.a(\a[16] ), .b(\b[15] ), .out0(new_n166));
  aoi112aa1n02x5               g071(.a(new_n166), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n166), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n168));
  norb02aa1n03x4               g073(.a(new_n168), .b(new_n167), .out0(\s[16] ));
  nano23aa1n02x5               g074(.a(new_n128), .b(new_n140), .c(new_n141), .d(new_n129), .out0(new_n170));
  nand23aa1d12x5               g075(.a(new_n159), .b(new_n165), .c(new_n166), .o1(new_n171));
  nano22aa1n03x7               g076(.a(new_n171), .b(new_n134), .c(new_n170), .out0(new_n172));
  aoai13aa1n12x5               g077(.a(new_n172), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n173));
  nanb03aa1n02x5               g078(.a(new_n161), .b(new_n166), .c(new_n165), .out0(new_n174));
  aob012aa1n02x5               g079(.a(new_n164), .b(\b[15] ), .c(\a[16] ), .out0(new_n175));
  oai112aa1n03x5               g080(.a(new_n174), .b(new_n175), .c(\b[15] ), .d(\a[16] ), .o1(new_n176));
  aoib12aa1n12x5               g081(.a(new_n176), .b(new_n146), .c(new_n171), .out0(new_n177));
  nand02aa1d08x5               g082(.a(new_n173), .b(new_n177), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g084(.a(\a[18] ), .o1(new_n180));
  inv000aa1d42x5               g085(.a(\a[17] ), .o1(new_n181));
  inv000aa1d42x5               g086(.a(\b[16] ), .o1(new_n182));
  oaoi03aa1n03x5               g087(.a(new_n181), .b(new_n182), .c(new_n178), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[17] ), .c(new_n180), .out0(\s[18] ));
  xroi22aa1d06x4               g089(.a(new_n181), .b(\b[16] ), .c(new_n180), .d(\b[17] ), .out0(new_n185));
  nanp02aa1n02x5               g090(.a(new_n182), .b(new_n181), .o1(new_n186));
  oaoi03aa1n02x5               g091(.a(\a[18] ), .b(\b[17] ), .c(new_n186), .o1(new_n187));
  nor002aa1d32x5               g092(.a(\b[18] ), .b(\a[19] ), .o1(new_n188));
  nand22aa1n03x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  norb02aa1n02x5               g094(.a(new_n189), .b(new_n188), .out0(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n187), .c(new_n178), .d(new_n185), .o1(new_n191));
  aoi112aa1n02x5               g096(.a(new_n190), .b(new_n187), .c(new_n178), .d(new_n185), .o1(new_n192));
  norb02aa1n02x7               g097(.a(new_n191), .b(new_n192), .out0(\s[19] ));
  xnrc02aa1n02x5               g098(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g099(.a(\b[19] ), .b(\a[20] ), .o1(new_n195));
  nand22aa1n06x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  nona22aa1n06x5               g102(.a(new_n191), .b(new_n197), .c(new_n188), .out0(new_n198));
  inv000aa1d42x5               g103(.a(new_n188), .o1(new_n199));
  aobi12aa1n06x5               g104(.a(new_n197), .b(new_n191), .c(new_n199), .out0(new_n200));
  norb02aa1n03x4               g105(.a(new_n198), .b(new_n200), .out0(\s[20] ));
  nano23aa1n06x5               g106(.a(new_n188), .b(new_n195), .c(new_n196), .d(new_n189), .out0(new_n202));
  nanp02aa1n02x5               g107(.a(new_n185), .b(new_n202), .o1(new_n203));
  oai022aa1n02x5               g108(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n204));
  oaib12aa1n09x5               g109(.a(new_n204), .b(new_n180), .c(\b[17] ), .out0(new_n205));
  nona23aa1n09x5               g110(.a(new_n196), .b(new_n189), .c(new_n188), .d(new_n195), .out0(new_n206));
  aoi012aa1n12x5               g111(.a(new_n195), .b(new_n188), .c(new_n196), .o1(new_n207));
  oai012aa1d24x5               g112(.a(new_n207), .b(new_n206), .c(new_n205), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  aoai13aa1n06x5               g114(.a(new_n209), .b(new_n203), .c(new_n173), .d(new_n177), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g116(.a(\b[20] ), .b(\a[21] ), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[21] ), .b(\b[20] ), .out0(new_n213));
  xorc02aa1n02x5               g118(.a(\a[22] ), .b(\b[21] ), .out0(new_n214));
  aoi112aa1n02x5               g119(.a(new_n212), .b(new_n214), .c(new_n210), .d(new_n213), .o1(new_n215));
  aoai13aa1n03x5               g120(.a(new_n214), .b(new_n212), .c(new_n210), .d(new_n213), .o1(new_n216));
  norb02aa1n02x7               g121(.a(new_n216), .b(new_n215), .out0(\s[22] ));
  inv000aa1d42x5               g122(.a(\a[21] ), .o1(new_n218));
  inv000aa1d42x5               g123(.a(\a[22] ), .o1(new_n219));
  xroi22aa1d06x4               g124(.a(new_n218), .b(\b[20] ), .c(new_n219), .d(\b[21] ), .out0(new_n220));
  nanp03aa1n02x5               g125(.a(new_n220), .b(new_n185), .c(new_n202), .o1(new_n221));
  inv000aa1d42x5               g126(.a(\b[21] ), .o1(new_n222));
  oaoi03aa1n12x5               g127(.a(new_n219), .b(new_n222), .c(new_n212), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  aoi012aa1n02x5               g129(.a(new_n224), .b(new_n208), .c(new_n220), .o1(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n221), .c(new_n173), .d(new_n177), .o1(new_n226));
  xorb03aa1n02x5               g131(.a(new_n226), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g132(.a(\b[22] ), .b(\a[23] ), .o1(new_n228));
  xorc02aa1n03x5               g133(.a(\a[23] ), .b(\b[22] ), .out0(new_n229));
  xorc02aa1n02x5               g134(.a(\a[24] ), .b(\b[23] ), .out0(new_n230));
  aoi112aa1n02x5               g135(.a(new_n228), .b(new_n230), .c(new_n226), .d(new_n229), .o1(new_n231));
  aoai13aa1n03x5               g136(.a(new_n230), .b(new_n228), .c(new_n226), .d(new_n229), .o1(new_n232));
  norb02aa1n02x7               g137(.a(new_n232), .b(new_n231), .out0(\s[24] ));
  and002aa1n02x5               g138(.a(new_n230), .b(new_n229), .o(new_n234));
  inv000aa1n02x5               g139(.a(new_n234), .o1(new_n235));
  nano32aa1n02x4               g140(.a(new_n235), .b(new_n220), .c(new_n185), .d(new_n202), .out0(new_n236));
  inv020aa1n02x5               g141(.a(new_n207), .o1(new_n237));
  aoai13aa1n03x5               g142(.a(new_n220), .b(new_n237), .c(new_n202), .d(new_n187), .o1(new_n238));
  aoi112aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n239));
  oab012aa1n02x4               g144(.a(new_n239), .b(\a[24] ), .c(\b[23] ), .out0(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n235), .c(new_n238), .d(new_n223), .o1(new_n241));
  tech160nm_fiaoi012aa1n05x5   g146(.a(new_n241), .b(new_n178), .c(new_n236), .o1(new_n242));
  xnrc02aa1n12x5               g147(.a(\b[24] ), .b(\a[25] ), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  xnrc02aa1n02x5               g149(.a(new_n242), .b(new_n244), .out0(\s[25] ));
  nor042aa1n03x5               g150(.a(\b[24] ), .b(\a[25] ), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  aoai13aa1n04x5               g152(.a(new_n244), .b(new_n241), .c(new_n178), .d(new_n236), .o1(new_n248));
  xnrc02aa1n06x5               g153(.a(\b[25] ), .b(\a[26] ), .out0(new_n249));
  nand43aa1n03x5               g154(.a(new_n248), .b(new_n247), .c(new_n249), .o1(new_n250));
  tech160nm_fiaoi012aa1n03p5x5 g155(.a(new_n249), .b(new_n248), .c(new_n247), .o1(new_n251));
  norb02aa1n03x4               g156(.a(new_n250), .b(new_n251), .out0(\s[26] ));
  nor042aa1n06x5               g157(.a(new_n249), .b(new_n243), .o1(new_n253));
  nano22aa1n03x7               g158(.a(new_n221), .b(new_n234), .c(new_n253), .out0(new_n254));
  nand02aa1d06x5               g159(.a(new_n178), .b(new_n254), .o1(new_n255));
  oao003aa1n02x5               g160(.a(\a[26] ), .b(\b[25] ), .c(new_n247), .carry(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  aoi012aa1n09x5               g162(.a(new_n257), .b(new_n241), .c(new_n253), .o1(new_n258));
  xorc02aa1n12x5               g163(.a(\a[27] ), .b(\b[26] ), .out0(new_n259));
  xnbna2aa1n03x5               g164(.a(new_n259), .b(new_n255), .c(new_n258), .out0(\s[27] ));
  norp02aa1n02x5               g165(.a(\b[26] ), .b(\a[27] ), .o1(new_n261));
  inv040aa1n03x5               g166(.a(new_n261), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n259), .o1(new_n263));
  tech160nm_fiaoi012aa1n03p5x5 g168(.a(new_n263), .b(new_n255), .c(new_n258), .o1(new_n264));
  xnrc02aa1n12x5               g169(.a(\b[27] ), .b(\a[28] ), .out0(new_n265));
  nano22aa1n03x5               g170(.a(new_n264), .b(new_n262), .c(new_n265), .out0(new_n266));
  aobi12aa1n06x5               g171(.a(new_n254), .b(new_n173), .c(new_n177), .out0(new_n267));
  aoai13aa1n04x5               g172(.a(new_n234), .b(new_n224), .c(new_n208), .d(new_n220), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n253), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n256), .b(new_n269), .c(new_n268), .d(new_n240), .o1(new_n270));
  oai012aa1n03x5               g175(.a(new_n259), .b(new_n270), .c(new_n267), .o1(new_n271));
  tech160nm_fiaoi012aa1n02p5x5 g176(.a(new_n265), .b(new_n271), .c(new_n262), .o1(new_n272));
  nor002aa1n02x5               g177(.a(new_n272), .b(new_n266), .o1(\s[28] ));
  norb02aa1d21x5               g178(.a(new_n259), .b(new_n265), .out0(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  tech160nm_fiaoi012aa1n05x5   g180(.a(new_n275), .b(new_n255), .c(new_n258), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[28] ), .b(\b[27] ), .c(new_n262), .carry(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[28] ), .b(\a[29] ), .out0(new_n278));
  nano22aa1n03x5               g183(.a(new_n276), .b(new_n277), .c(new_n278), .out0(new_n279));
  oaih12aa1n02x5               g184(.a(new_n274), .b(new_n270), .c(new_n267), .o1(new_n280));
  aoi012aa1n03x5               g185(.a(new_n278), .b(new_n280), .c(new_n277), .o1(new_n281));
  norp02aa1n03x5               g186(.a(new_n281), .b(new_n279), .o1(\s[29] ));
  xorb03aa1n02x5               g187(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g188(.a(new_n259), .b(new_n278), .c(new_n265), .out0(new_n284));
  inv000aa1d42x5               g189(.a(new_n284), .o1(new_n285));
  tech160nm_fiaoi012aa1n02p5x5 g190(.a(new_n285), .b(new_n255), .c(new_n258), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[29] ), .b(\b[28] ), .c(new_n277), .carry(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[29] ), .b(\a[30] ), .out0(new_n288));
  nano22aa1n03x5               g193(.a(new_n286), .b(new_n287), .c(new_n288), .out0(new_n289));
  oaih12aa1n02x5               g194(.a(new_n284), .b(new_n270), .c(new_n267), .o1(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n288), .b(new_n290), .c(new_n287), .o1(new_n291));
  norp02aa1n03x5               g196(.a(new_n291), .b(new_n289), .o1(\s[30] ));
  norb02aa1n02x5               g197(.a(new_n284), .b(new_n288), .out0(new_n293));
  inv000aa1n02x5               g198(.a(new_n293), .o1(new_n294));
  tech160nm_fiaoi012aa1n03p5x5 g199(.a(new_n294), .b(new_n255), .c(new_n258), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[30] ), .b(\b[29] ), .c(new_n287), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[30] ), .b(\a[31] ), .out0(new_n297));
  nano22aa1n03x5               g202(.a(new_n295), .b(new_n296), .c(new_n297), .out0(new_n298));
  oaih12aa1n02x5               g203(.a(new_n293), .b(new_n270), .c(new_n267), .o1(new_n299));
  aoi012aa1n03x5               g204(.a(new_n297), .b(new_n299), .c(new_n296), .o1(new_n300));
  norp02aa1n03x5               g205(.a(new_n300), .b(new_n298), .o1(\s[31] ));
  xnrb03aa1n02x5               g206(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g207(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n303));
  xorb03aa1n02x5               g208(.a(new_n303), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g209(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaib12aa1n02x5               g210(.a(new_n119), .b(new_n116), .c(new_n109), .out0(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoib12aa1n02x5               g212(.a(new_n120), .b(new_n306), .c(new_n115), .out0(new_n308));
  xnrb03aa1n02x5               g213(.a(new_n308), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g214(.a(\a[7] ), .b(\b[6] ), .c(new_n308), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g216(.a(new_n123), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


