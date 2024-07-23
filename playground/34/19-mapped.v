// Benchmark "adder" written by ABC on Thu Jul 18 05:32:02 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n312, new_n315, new_n316, new_n318, new_n319, new_n321,
    new_n322, new_n323, new_n325;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand42aa1d28x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  nor042aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n06x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nand22aa1n06x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aoi012aa1n09x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  nor002aa1n16x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand02aa1n08x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor022aa1n16x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n04x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n09x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  tech160nm_fiaoi012aa1n03p5x5 g013(.a(new_n104), .b(new_n106), .c(new_n105), .o1(new_n109));
  oai012aa1n12x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .o1(new_n110));
  nor002aa1n08x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1n03x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  tech160nm_fixnrc02aa1n05x5   g020(.a(\b[5] ), .b(\a[6] ), .out0(new_n116));
  xnrc02aa1n12x5               g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  nor043aa1n03x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\a[6] ), .o1(new_n119));
  aoi112aa1n03x5               g024(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n120));
  aoib12aa1n06x5               g025(.a(new_n120), .b(new_n119), .c(\b[5] ), .out0(new_n121));
  tech160nm_fiaoi012aa1n03p5x5 g026(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n122));
  tech160nm_fioai012aa1n05x5   g027(.a(new_n122), .b(new_n115), .c(new_n121), .o1(new_n123));
  aoai13aa1n02x7               g028(.a(new_n99), .b(new_n123), .c(new_n110), .d(new_n118), .o1(new_n124));
  nor042aa1n06x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand42aa1n16x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n124), .c(new_n98), .out0(\s[10] ));
  inv000aa1n02x5               g033(.a(new_n127), .o1(new_n129));
  oai012aa1n12x5               g034(.a(new_n126), .b(new_n125), .c(new_n97), .o1(new_n130));
  aoai13aa1n06x5               g035(.a(new_n130), .b(new_n129), .c(new_n124), .d(new_n98), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1n03x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  inv000aa1d42x5               g039(.a(new_n134), .o1(new_n135));
  nona22aa1n02x4               g040(.a(new_n131), .b(new_n133), .c(new_n135), .out0(new_n136));
  aoi012aa1n02x5               g041(.a(new_n133), .b(new_n131), .c(new_n134), .o1(new_n137));
  nor002aa1n06x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand42aa1n08x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  inv000aa1n02x5               g045(.a(new_n138), .o1(new_n141));
  oai112aa1n02x5               g046(.a(new_n141), .b(new_n139), .c(\b[10] ), .d(\a[11] ), .o1(new_n142));
  obai22aa1n02x5               g047(.a(new_n136), .b(new_n142), .c(new_n137), .d(new_n140), .out0(\s[12] ));
  nano23aa1n03x7               g048(.a(new_n133), .b(new_n138), .c(new_n139), .d(new_n134), .out0(new_n144));
  nano23aa1n03x7               g049(.a(new_n97), .b(new_n125), .c(new_n126), .d(new_n99), .out0(new_n145));
  and002aa1n02x5               g050(.a(new_n145), .b(new_n144), .o(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n123), .c(new_n110), .d(new_n118), .o1(new_n147));
  inv000aa1n02x5               g052(.a(new_n133), .o1(new_n148));
  aoai13aa1n06x5               g053(.a(new_n141), .b(new_n135), .c(new_n130), .d(new_n148), .o1(new_n149));
  aob012aa1n06x5               g054(.a(new_n147), .b(new_n149), .c(new_n139), .out0(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  xnrc02aa1n12x5               g056(.a(\b[12] ), .b(\a[13] ), .out0(new_n152));
  nanb02aa1n02x5               g057(.a(new_n152), .b(new_n150), .out0(new_n153));
  inv000aa1d42x5               g058(.a(\a[14] ), .o1(new_n154));
  inv020aa1d32x5               g059(.a(\b[13] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(new_n155), .b(new_n154), .o1(new_n156));
  nor042aa1d18x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  xnrc02aa1n12x5               g062(.a(\b[13] ), .b(\a[14] ), .out0(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  nor042aa1n04x5               g064(.a(new_n158), .b(new_n152), .o1(new_n160));
  oaoi03aa1n12x5               g065(.a(new_n154), .b(new_n155), .c(new_n157), .o1(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  aoi012aa1n06x5               g067(.a(new_n162), .b(new_n150), .c(new_n160), .o1(new_n163));
  aboi22aa1n03x5               g068(.a(new_n163), .b(new_n156), .c(new_n153), .d(new_n159), .out0(\s[14] ));
  nor002aa1d32x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  nand22aa1n09x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n163), .b(new_n167), .c(new_n166), .out0(\s[15] ));
  norb02aa1n02x5               g073(.a(new_n167), .b(new_n165), .out0(new_n169));
  aoai13aa1n06x5               g074(.a(new_n169), .b(new_n162), .c(new_n150), .d(new_n160), .o1(new_n170));
  norp02aa1n24x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanp02aa1n09x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  norb03aa1n02x5               g078(.a(new_n172), .b(new_n165), .c(new_n171), .out0(new_n174));
  nanp02aa1n02x5               g079(.a(new_n170), .b(new_n174), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n173), .c(new_n170), .d(new_n166), .o1(\s[16] ));
  inv000aa1n02x5               g081(.a(new_n160), .o1(new_n177));
  nano23aa1n06x5               g082(.a(new_n165), .b(new_n171), .c(new_n172), .d(new_n167), .out0(new_n178));
  nano32aa1n03x7               g083(.a(new_n177), .b(new_n178), .c(new_n144), .d(new_n145), .out0(new_n179));
  aoai13aa1n06x5               g084(.a(new_n179), .b(new_n123), .c(new_n110), .d(new_n118), .o1(new_n180));
  nona23aa1n09x5               g085(.a(new_n172), .b(new_n167), .c(new_n165), .d(new_n171), .out0(new_n181));
  nor043aa1n02x5               g086(.a(new_n181), .b(new_n158), .c(new_n152), .o1(new_n182));
  tech160nm_fioai012aa1n03p5x5 g087(.a(new_n172), .b(new_n171), .c(new_n165), .o1(new_n183));
  oaih12aa1n02x5               g088(.a(new_n183), .b(new_n181), .c(new_n161), .o1(new_n184));
  aoi013aa1n06x4               g089(.a(new_n184), .b(new_n182), .c(new_n149), .d(new_n139), .o1(new_n185));
  nanp02aa1n09x5               g090(.a(new_n180), .b(new_n185), .o1(new_n186));
  xorc02aa1n12x5               g091(.a(\a[17] ), .b(\b[16] ), .out0(new_n187));
  aoai13aa1n02x5               g092(.a(new_n178), .b(new_n162), .c(new_n150), .d(new_n160), .o1(new_n188));
  norb02aa1n02x5               g093(.a(new_n183), .b(new_n187), .out0(new_n189));
  aoi022aa1n02x5               g094(.a(new_n188), .b(new_n189), .c(new_n187), .d(new_n186), .o1(\s[17] ));
  nor002aa1d32x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  inv030aa1d32x5               g096(.a(\a[17] ), .o1(new_n192));
  inv000aa1d48x5               g097(.a(\b[16] ), .o1(new_n193));
  nand02aa1d28x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  aboi22aa1n03x5               g099(.a(new_n191), .b(new_n194), .c(new_n192), .d(new_n193), .out0(new_n195));
  aob012aa1n02x5               g100(.a(new_n195), .b(new_n186), .c(new_n187), .out0(new_n196));
  nano22aa1n03x7               g101(.a(new_n191), .b(new_n187), .c(new_n194), .out0(new_n197));
  aoi013aa1n09x5               g102(.a(new_n191), .b(new_n194), .c(new_n192), .d(new_n193), .o1(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  aoi012aa1n06x5               g104(.a(new_n199), .b(new_n186), .c(new_n197), .o1(new_n200));
  oa0012aa1n03x5               g105(.a(new_n196), .b(new_n200), .c(new_n191), .o(\s[18] ));
  nor002aa1d32x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  nand42aa1n06x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  xnbna2aa1n03x5               g109(.a(new_n200), .b(new_n204), .c(new_n203), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanb02aa1n02x5               g111(.a(new_n202), .b(new_n204), .out0(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n199), .c(new_n186), .d(new_n197), .o1(new_n209));
  nor002aa1d24x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nand02aa1d20x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n211), .o1(new_n213));
  oai022aa1n02x5               g118(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n214));
  nona22aa1n03x5               g119(.a(new_n209), .b(new_n213), .c(new_n214), .out0(new_n215));
  aoai13aa1n03x5               g120(.a(new_n215), .b(new_n212), .c(new_n203), .d(new_n209), .o1(\s[20] ));
  nona23aa1d18x5               g121(.a(new_n211), .b(new_n204), .c(new_n202), .d(new_n210), .out0(new_n217));
  nano23aa1d12x5               g122(.a(new_n217), .b(new_n191), .c(new_n187), .d(new_n194), .out0(new_n218));
  tech160nm_fioai012aa1n04x5   g123(.a(new_n211), .b(new_n210), .c(new_n202), .o1(new_n219));
  oai012aa1n12x5               g124(.a(new_n219), .b(new_n217), .c(new_n198), .o1(new_n220));
  xnrc02aa1n12x5               g125(.a(\b[20] ), .b(\a[21] ), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  aoai13aa1n06x5               g127(.a(new_n222), .b(new_n220), .c(new_n186), .d(new_n218), .o1(new_n223));
  aoi112aa1n02x5               g128(.a(new_n222), .b(new_n220), .c(new_n186), .d(new_n218), .o1(new_n224));
  norb02aa1n03x4               g129(.a(new_n223), .b(new_n224), .out0(\s[21] ));
  orn002aa1n02x5               g130(.a(\a[21] ), .b(\b[20] ), .o(new_n226));
  xorc02aa1n03x5               g131(.a(\a[22] ), .b(\b[21] ), .out0(new_n227));
  and002aa1n02x5               g132(.a(\b[21] ), .b(\a[22] ), .o(new_n228));
  oai022aa1n02x5               g133(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n229));
  nona22aa1n03x5               g134(.a(new_n223), .b(new_n228), .c(new_n229), .out0(new_n230));
  aoai13aa1n03x5               g135(.a(new_n230), .b(new_n227), .c(new_n226), .d(new_n223), .o1(\s[22] ));
  norb02aa1n09x5               g136(.a(new_n227), .b(new_n221), .out0(new_n232));
  nano22aa1n03x5               g137(.a(new_n217), .b(new_n197), .c(new_n232), .out0(new_n233));
  oaoi03aa1n03x5               g138(.a(\a[22] ), .b(\b[21] ), .c(new_n226), .o1(new_n234));
  tech160nm_fiao0012aa1n02p5x5 g139(.a(new_n234), .b(new_n220), .c(new_n232), .o(new_n235));
  xnrc02aa1n12x5               g140(.a(\b[22] ), .b(\a[23] ), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n235), .c(new_n186), .d(new_n233), .o1(new_n238));
  aoi112aa1n02x5               g143(.a(new_n237), .b(new_n235), .c(new_n186), .d(new_n233), .o1(new_n239));
  norb02aa1n03x4               g144(.a(new_n238), .b(new_n239), .out0(\s[23] ));
  norp02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  xorc02aa1n02x5               g147(.a(\a[24] ), .b(\b[23] ), .out0(new_n243));
  and002aa1n02x5               g148(.a(\b[23] ), .b(\a[24] ), .o(new_n244));
  oai022aa1n02x5               g149(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n245));
  nona22aa1n03x5               g150(.a(new_n238), .b(new_n244), .c(new_n245), .out0(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n243), .c(new_n242), .d(new_n238), .o1(\s[24] ));
  inv000aa1n02x5               g152(.a(new_n218), .o1(new_n248));
  norb02aa1n09x5               g153(.a(new_n243), .b(new_n236), .out0(new_n249));
  nano22aa1n02x5               g154(.a(new_n248), .b(new_n232), .c(new_n249), .out0(new_n250));
  aoai13aa1n04x5               g155(.a(new_n249), .b(new_n234), .c(new_n220), .d(new_n232), .o1(new_n251));
  aob012aa1n02x5               g156(.a(new_n245), .b(\b[23] ), .c(\a[24] ), .out0(new_n252));
  nanp02aa1n03x5               g157(.a(new_n251), .b(new_n252), .o1(new_n253));
  xorc02aa1n12x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n253), .c(new_n186), .d(new_n250), .o1(new_n255));
  aoi112aa1n02x5               g160(.a(new_n254), .b(new_n253), .c(new_n186), .d(new_n250), .o1(new_n256));
  norb02aa1n03x4               g161(.a(new_n255), .b(new_n256), .out0(\s[25] ));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  tech160nm_fixorc02aa1n05x5   g164(.a(\a[26] ), .b(\b[25] ), .out0(new_n260));
  and002aa1n02x5               g165(.a(\b[25] ), .b(\a[26] ), .o(new_n261));
  oai022aa1n02x5               g166(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n262));
  nona22aa1n03x5               g167(.a(new_n255), .b(new_n261), .c(new_n262), .out0(new_n263));
  aoai13aa1n03x5               g168(.a(new_n263), .b(new_n260), .c(new_n259), .d(new_n255), .o1(\s[26] ));
  inv000aa1n02x5               g169(.a(new_n232), .o1(new_n265));
  and002aa1n02x7               g170(.a(new_n260), .b(new_n254), .o(new_n266));
  inv000aa1n02x5               g171(.a(new_n266), .o1(new_n267));
  nona23aa1n09x5               g172(.a(new_n249), .b(new_n218), .c(new_n267), .d(new_n265), .out0(new_n268));
  nanb02aa1n06x5               g173(.a(new_n268), .b(new_n186), .out0(new_n269));
  aob012aa1n02x5               g174(.a(new_n262), .b(\b[25] ), .c(\a[26] ), .out0(new_n270));
  aobi12aa1n03x5               g175(.a(new_n270), .b(new_n253), .c(new_n266), .out0(new_n271));
  xorc02aa1n02x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  xnbna2aa1n03x5               g177(.a(new_n272), .b(new_n271), .c(new_n269), .out0(\s[27] ));
  norp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  aoi012aa1n09x5               g180(.a(new_n268), .b(new_n180), .c(new_n185), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n270), .b(new_n267), .c(new_n251), .d(new_n252), .o1(new_n277));
  tech160nm_fioai012aa1n05x5   g182(.a(new_n272), .b(new_n277), .c(new_n276), .o1(new_n278));
  xorc02aa1n02x5               g183(.a(\a[28] ), .b(\b[27] ), .out0(new_n279));
  oai022aa1d24x5               g184(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n280));
  aoi012aa1n02x5               g185(.a(new_n280), .b(\a[28] ), .c(\b[27] ), .o1(new_n281));
  nanp02aa1n03x5               g186(.a(new_n278), .b(new_n281), .o1(new_n282));
  aoai13aa1n03x5               g187(.a(new_n282), .b(new_n279), .c(new_n278), .d(new_n275), .o1(\s[28] ));
  xorc02aa1n02x5               g188(.a(\a[29] ), .b(\b[28] ), .out0(new_n284));
  and002aa1n02x5               g189(.a(new_n279), .b(new_n272), .o(new_n285));
  oai012aa1n04x7               g190(.a(new_n285), .b(new_n277), .c(new_n276), .o1(new_n286));
  inv000aa1d42x5               g191(.a(\b[27] ), .o1(new_n287));
  oaib12aa1n09x5               g192(.a(new_n280), .b(new_n287), .c(\a[28] ), .out0(new_n288));
  nand03aa1n02x5               g193(.a(new_n286), .b(new_n288), .c(new_n284), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n288), .o1(new_n290));
  oaoi13aa1n02x7               g195(.a(new_n290), .b(new_n285), .c(new_n277), .d(new_n276), .o1(new_n291));
  oai012aa1n03x5               g196(.a(new_n289), .b(new_n291), .c(new_n284), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g198(.a(new_n272), .b(new_n284), .c(new_n279), .o(new_n294));
  tech160nm_fioaoi03aa1n03p5x5 g199(.a(\a[29] ), .b(\b[28] ), .c(new_n288), .o1(new_n295));
  oaoi13aa1n02x7               g200(.a(new_n295), .b(new_n294), .c(new_n277), .d(new_n276), .o1(new_n296));
  xorc02aa1n02x5               g201(.a(\a[30] ), .b(\b[29] ), .out0(new_n297));
  oaih12aa1n02x5               g202(.a(new_n294), .b(new_n277), .c(new_n276), .o1(new_n298));
  norb02aa1n03x5               g203(.a(new_n297), .b(new_n295), .out0(new_n299));
  nand02aa1n02x5               g204(.a(new_n298), .b(new_n299), .o1(new_n300));
  oai012aa1n03x5               g205(.a(new_n300), .b(new_n296), .c(new_n297), .o1(\s[30] ));
  xorc02aa1n02x5               g206(.a(\a[31] ), .b(\b[30] ), .out0(new_n302));
  and003aa1n02x5               g207(.a(new_n285), .b(new_n297), .c(new_n284), .o(new_n303));
  oaih12aa1n02x5               g208(.a(new_n303), .b(new_n277), .c(new_n276), .o1(new_n304));
  aoi012aa1n02x7               g209(.a(new_n299), .b(\a[30] ), .c(\b[29] ), .o1(new_n305));
  norb02aa1n02x5               g210(.a(new_n302), .b(new_n305), .out0(new_n306));
  nand02aa1n02x5               g211(.a(new_n304), .b(new_n306), .o1(new_n307));
  oaoi13aa1n02x7               g212(.a(new_n305), .b(new_n303), .c(new_n277), .d(new_n276), .o1(new_n308));
  oai012aa1n03x5               g213(.a(new_n307), .b(new_n308), .c(new_n302), .o1(\s[31] ));
  xnrb03aa1n02x5               g214(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nanb02aa1n02x5               g215(.a(new_n104), .b(new_n105), .out0(new_n311));
  oaib12aa1n02x5               g216(.a(new_n107), .b(new_n106), .c(new_n103), .out0(new_n312));
  aboi22aa1n03x5               g217(.a(new_n104), .b(new_n110), .c(new_n312), .d(new_n311), .out0(\s[4] ));
  xorb03aa1n02x5               g218(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g219(.a(\a[5] ), .b(\b[4] ), .o(new_n315));
  nanb02aa1n02x5               g220(.a(new_n117), .b(new_n110), .out0(new_n316));
  xobna2aa1n03x5               g221(.a(new_n116), .b(new_n316), .c(new_n315), .out0(\s[6] ));
  norp02aa1n02x5               g222(.a(new_n117), .b(new_n116), .o1(new_n318));
  aob012aa1n02x5               g223(.a(new_n121), .b(new_n110), .c(new_n318), .out0(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanb02aa1n02x5               g225(.a(new_n111), .b(new_n112), .out0(new_n321));
  inv000aa1d42x5               g226(.a(new_n113), .o1(new_n322));
  nanp03aa1n02x5               g227(.a(new_n319), .b(new_n322), .c(new_n114), .o1(new_n323));
  xobna2aa1n03x5               g228(.a(new_n321), .b(new_n323), .c(new_n322), .out0(\s[8] ));
  aoi012aa1n02x5               g229(.a(new_n123), .b(new_n110), .c(new_n118), .o1(new_n325));
  xnbna2aa1n03x5               g230(.a(new_n325), .b(new_n98), .c(new_n99), .out0(\s[9] ));
endmodule


