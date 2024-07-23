// Benchmark "adder" written by ABC on Wed Jul 17 18:06:27 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n308, new_n311, new_n313,
    new_n315;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  norp02aa1n24x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nand22aa1n03x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nor022aa1n16x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nand22aa1n03x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nona23aa1n09x5               g006(.a(new_n101), .b(new_n99), .c(new_n98), .d(new_n100), .out0(new_n102));
  tech160nm_fixnrc02aa1n04x5   g007(.a(\b[5] ), .b(\a[6] ), .out0(new_n103));
  inv000aa1d42x5               g008(.a(\a[5] ), .o1(new_n104));
  nanb02aa1d24x5               g009(.a(\b[4] ), .b(new_n104), .out0(new_n105));
  nanp02aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  nano22aa1n02x5               g011(.a(new_n103), .b(new_n105), .c(new_n106), .out0(new_n107));
  and002aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o(new_n108));
  oa0022aa1n06x5               g013(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n109));
  inv040aa1d32x5               g014(.a(\a[3] ), .o1(new_n110));
  inv000aa1d42x5               g015(.a(\b[2] ), .o1(new_n111));
  nanp02aa1n04x5               g016(.a(new_n111), .b(new_n110), .o1(new_n112));
  nand02aa1n03x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nand02aa1n02x5               g018(.a(new_n112), .b(new_n113), .o1(new_n114));
  nor042aa1n02x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  nand42aa1n06x5               g020(.a(\b[1] ), .b(\a[2] ), .o1(new_n116));
  nand02aa1n10x5               g021(.a(\b[0] ), .b(\a[1] ), .o1(new_n117));
  aoi012aa1n12x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  tech160nm_fioai012aa1n04x5   g023(.a(new_n109), .b(new_n118), .c(new_n114), .o1(new_n119));
  nona23aa1n06x5               g024(.a(new_n119), .b(new_n107), .c(new_n102), .d(new_n108), .out0(new_n120));
  nano23aa1n06x5               g025(.a(new_n98), .b(new_n100), .c(new_n101), .d(new_n99), .out0(new_n121));
  aoi112aa1n02x5               g026(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n122));
  oaoi03aa1n12x5               g027(.a(\a[6] ), .b(\b[5] ), .c(new_n105), .o1(new_n123));
  aoi112aa1n03x5               g028(.a(new_n122), .b(new_n98), .c(new_n121), .d(new_n123), .o1(new_n124));
  xorc02aa1n12x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  inv000aa1d42x5               g030(.a(new_n125), .o1(new_n126));
  aoai13aa1n06x5               g031(.a(new_n97), .b(new_n126), .c(new_n120), .d(new_n124), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  tech160nm_fixorc02aa1n05x5   g033(.a(\a[10] ), .b(\b[9] ), .out0(new_n129));
  nor002aa1d32x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nand42aa1n06x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  oai022aa1d24x5               g037(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n133));
  aob012aa1d24x5               g038(.a(new_n133), .b(\b[9] ), .c(\a[10] ), .out0(new_n134));
  inv000aa1d42x5               g039(.a(new_n134), .o1(new_n135));
  aoai13aa1n03x5               g040(.a(new_n132), .b(new_n135), .c(new_n127), .d(new_n129), .o1(new_n136));
  aoi112aa1n02x5               g041(.a(new_n132), .b(new_n135), .c(new_n127), .d(new_n129), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(\s[11] ));
  inv000aa1d42x5               g043(.a(new_n130), .o1(new_n139));
  nor002aa1n16x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand02aa1n08x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n136), .c(new_n139), .out0(\s[12] ));
  nano23aa1n09x5               g048(.a(new_n130), .b(new_n140), .c(new_n141), .d(new_n131), .out0(new_n144));
  nand23aa1n09x5               g049(.a(new_n144), .b(new_n129), .c(new_n125), .o1(new_n145));
  nona23aa1d18x5               g050(.a(new_n141), .b(new_n131), .c(new_n130), .d(new_n140), .out0(new_n146));
  tech160nm_fioai012aa1n03p5x5 g051(.a(new_n141), .b(new_n140), .c(new_n130), .o1(new_n147));
  oai012aa1d24x5               g052(.a(new_n147), .b(new_n146), .c(new_n134), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  aoai13aa1n03x5               g054(.a(new_n149), .b(new_n145), .c(new_n120), .d(new_n124), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n12x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand42aa1n08x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n152), .b(new_n150), .c(new_n153), .o1(new_n154));
  xnrb03aa1n03x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xorc02aa1n02x5               g060(.a(\a[5] ), .b(\b[4] ), .out0(new_n156));
  norb03aa1n03x5               g061(.a(new_n156), .b(new_n102), .c(new_n103), .out0(new_n157));
  oaoi13aa1n09x5               g062(.a(new_n108), .b(new_n109), .c(new_n118), .d(new_n114), .o1(new_n158));
  nanp02aa1n03x5               g063(.a(new_n121), .b(new_n123), .o1(new_n159));
  nona22aa1n02x4               g064(.a(new_n159), .b(new_n122), .c(new_n98), .out0(new_n160));
  inv000aa1n02x5               g065(.a(new_n145), .o1(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n160), .c(new_n158), .d(new_n157), .o1(new_n162));
  norp02aa1n24x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nand42aa1n16x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nona23aa1n03x5               g069(.a(new_n164), .b(new_n153), .c(new_n152), .d(new_n163), .out0(new_n165));
  oai012aa1n03x5               g070(.a(new_n164), .b(new_n163), .c(new_n152), .o1(new_n166));
  aoai13aa1n02x7               g071(.a(new_n166), .b(new_n165), .c(new_n162), .d(new_n149), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  tech160nm_fixnrc02aa1n04x5   g074(.a(\b[14] ), .b(\a[15] ), .out0(new_n170));
  inv000aa1n02x5               g075(.a(new_n170), .o1(new_n171));
  xnrc02aa1n12x5               g076(.a(\b[15] ), .b(\a[16] ), .out0(new_n172));
  inv000aa1n02x5               g077(.a(new_n172), .o1(new_n173));
  aoi112aa1n02x5               g078(.a(new_n173), .b(new_n169), .c(new_n167), .d(new_n171), .o1(new_n174));
  aoai13aa1n03x5               g079(.a(new_n173), .b(new_n169), .c(new_n167), .d(new_n171), .o1(new_n175));
  norb02aa1n02x7               g080(.a(new_n175), .b(new_n174), .out0(\s[16] ));
  nano23aa1n02x5               g081(.a(new_n152), .b(new_n163), .c(new_n164), .d(new_n153), .out0(new_n177));
  nano32aa1n03x7               g082(.a(new_n145), .b(new_n173), .c(new_n171), .d(new_n177), .out0(new_n178));
  aoai13aa1n06x5               g083(.a(new_n178), .b(new_n160), .c(new_n158), .d(new_n157), .o1(new_n179));
  nor043aa1n02x5               g084(.a(new_n165), .b(new_n172), .c(new_n170), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n181));
  nor042aa1n02x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(new_n182), .o1(new_n183));
  oai013aa1n02x4               g088(.a(new_n183), .b(new_n170), .c(new_n172), .d(new_n166), .o1(new_n184));
  aoi112aa1n06x5               g089(.a(new_n184), .b(new_n181), .c(new_n148), .d(new_n180), .o1(new_n185));
  nand02aa1d08x5               g090(.a(new_n179), .b(new_n185), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g092(.a(\a[18] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\a[17] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\b[16] ), .o1(new_n190));
  oaoi03aa1n03x5               g095(.a(new_n189), .b(new_n190), .c(new_n186), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n188), .out0(\s[18] ));
  xroi22aa1d04x5               g097(.a(new_n189), .b(\b[16] ), .c(new_n188), .d(\b[17] ), .out0(new_n193));
  nanp02aa1n02x5               g098(.a(new_n190), .b(new_n189), .o1(new_n194));
  oaoi03aa1n02x5               g099(.a(\a[18] ), .b(\b[17] ), .c(new_n194), .o1(new_n195));
  nor042aa1n04x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nanp02aa1n04x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n195), .c(new_n186), .d(new_n193), .o1(new_n199));
  aoi112aa1n02x5               g104(.a(new_n198), .b(new_n195), .c(new_n186), .d(new_n193), .o1(new_n200));
  norb02aa1n02x7               g105(.a(new_n199), .b(new_n200), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nand02aa1n04x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  nona22aa1n02x5               g110(.a(new_n199), .b(new_n205), .c(new_n196), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n205), .o1(new_n207));
  oaoi13aa1n06x5               g112(.a(new_n207), .b(new_n199), .c(\a[19] ), .d(\b[18] ), .o1(new_n208));
  norb02aa1n03x4               g113(.a(new_n206), .b(new_n208), .out0(\s[20] ));
  nano23aa1n09x5               g114(.a(new_n196), .b(new_n203), .c(new_n204), .d(new_n197), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n193), .b(new_n210), .o1(new_n211));
  oai022aa1n02x7               g116(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n212));
  oaib12aa1n06x5               g117(.a(new_n212), .b(new_n188), .c(\b[17] ), .out0(new_n213));
  nona23aa1n09x5               g118(.a(new_n204), .b(new_n197), .c(new_n196), .d(new_n203), .out0(new_n214));
  tech160nm_fiaoi012aa1n05x5   g119(.a(new_n203), .b(new_n196), .c(new_n204), .o1(new_n215));
  oai012aa1n18x5               g120(.a(new_n215), .b(new_n214), .c(new_n213), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  aoai13aa1n04x5               g122(.a(new_n217), .b(new_n211), .c(new_n179), .d(new_n185), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  xorc02aa1n02x5               g125(.a(\a[21] ), .b(\b[20] ), .out0(new_n221));
  xorc02aa1n02x5               g126(.a(\a[22] ), .b(\b[21] ), .out0(new_n222));
  aoi112aa1n02x5               g127(.a(new_n220), .b(new_n222), .c(new_n218), .d(new_n221), .o1(new_n223));
  aoai13aa1n03x5               g128(.a(new_n222), .b(new_n220), .c(new_n218), .d(new_n221), .o1(new_n224));
  norb02aa1n02x7               g129(.a(new_n224), .b(new_n223), .out0(\s[22] ));
  inv000aa1d42x5               g130(.a(\a[21] ), .o1(new_n226));
  inv000aa1d42x5               g131(.a(\a[22] ), .o1(new_n227));
  xroi22aa1d06x4               g132(.a(new_n226), .b(\b[20] ), .c(new_n227), .d(\b[21] ), .out0(new_n228));
  nanp03aa1n02x5               g133(.a(new_n228), .b(new_n193), .c(new_n210), .o1(new_n229));
  inv000aa1d42x5               g134(.a(\b[21] ), .o1(new_n230));
  oaoi03aa1n02x5               g135(.a(new_n227), .b(new_n230), .c(new_n220), .o1(new_n231));
  inv000aa1n02x5               g136(.a(new_n231), .o1(new_n232));
  tech160nm_fiaoi012aa1n03p5x5 g137(.a(new_n232), .b(new_n216), .c(new_n228), .o1(new_n233));
  aoai13aa1n04x5               g138(.a(new_n233), .b(new_n229), .c(new_n179), .d(new_n185), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  tech160nm_fixorc02aa1n02p5x5 g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  xorc02aa1n03x5               g142(.a(\a[24] ), .b(\b[23] ), .out0(new_n238));
  aoi112aa1n02x5               g143(.a(new_n236), .b(new_n238), .c(new_n234), .d(new_n237), .o1(new_n239));
  aoai13aa1n03x5               g144(.a(new_n238), .b(new_n236), .c(new_n234), .d(new_n237), .o1(new_n240));
  norb02aa1n02x7               g145(.a(new_n240), .b(new_n239), .out0(\s[24] ));
  and002aa1n02x5               g146(.a(new_n238), .b(new_n237), .o(new_n242));
  inv000aa1n02x5               g147(.a(new_n242), .o1(new_n243));
  nano32aa1n02x4               g148(.a(new_n243), .b(new_n228), .c(new_n193), .d(new_n210), .out0(new_n244));
  inv000aa1n02x5               g149(.a(new_n215), .o1(new_n245));
  aoai13aa1n03x5               g150(.a(new_n228), .b(new_n245), .c(new_n210), .d(new_n195), .o1(new_n246));
  orn002aa1n02x5               g151(.a(\a[23] ), .b(\b[22] ), .o(new_n247));
  oao003aa1n02x5               g152(.a(\a[24] ), .b(\b[23] ), .c(new_n247), .carry(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n243), .c(new_n246), .d(new_n231), .o1(new_n249));
  tech160nm_fixorc02aa1n05x5   g154(.a(\a[25] ), .b(\b[24] ), .out0(new_n250));
  aoai13aa1n06x5               g155(.a(new_n250), .b(new_n249), .c(new_n186), .d(new_n244), .o1(new_n251));
  aoi112aa1n02x5               g156(.a(new_n250), .b(new_n249), .c(new_n186), .d(new_n244), .o1(new_n252));
  norb02aa1n02x7               g157(.a(new_n251), .b(new_n252), .out0(\s[25] ));
  nor042aa1n03x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  tech160nm_fixorc02aa1n02p5x5 g159(.a(\a[26] ), .b(\b[25] ), .out0(new_n255));
  nona22aa1n02x5               g160(.a(new_n251), .b(new_n255), .c(new_n254), .out0(new_n256));
  inv000aa1d42x5               g161(.a(new_n254), .o1(new_n257));
  aobi12aa1n06x5               g162(.a(new_n255), .b(new_n251), .c(new_n257), .out0(new_n258));
  norb02aa1n03x4               g163(.a(new_n256), .b(new_n258), .out0(\s[26] ));
  nanp02aa1n04x5               g164(.a(new_n120), .b(new_n124), .o1(new_n260));
  nand42aa1n02x5               g165(.a(new_n148), .b(new_n180), .o1(new_n261));
  nona22aa1n02x4               g166(.a(new_n261), .b(new_n184), .c(new_n181), .out0(new_n262));
  and002aa1n06x5               g167(.a(new_n255), .b(new_n250), .o(new_n263));
  nano22aa1n03x7               g168(.a(new_n229), .b(new_n242), .c(new_n263), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n262), .c(new_n260), .d(new_n178), .o1(new_n265));
  oao003aa1n02x5               g170(.a(\a[26] ), .b(\b[25] ), .c(new_n257), .carry(new_n266));
  aobi12aa1n06x5               g171(.a(new_n266), .b(new_n249), .c(new_n263), .out0(new_n267));
  xorc02aa1n12x5               g172(.a(\a[27] ), .b(\b[26] ), .out0(new_n268));
  xnbna2aa1n03x5               g173(.a(new_n268), .b(new_n267), .c(new_n265), .out0(\s[27] ));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  inv040aa1n03x5               g175(.a(new_n270), .o1(new_n271));
  aobi12aa1n06x5               g176(.a(new_n268), .b(new_n267), .c(new_n265), .out0(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[27] ), .b(\a[28] ), .out0(new_n273));
  nano22aa1n03x7               g178(.a(new_n272), .b(new_n271), .c(new_n273), .out0(new_n274));
  aobi12aa1n06x5               g179(.a(new_n264), .b(new_n179), .c(new_n185), .out0(new_n275));
  aoai13aa1n09x5               g180(.a(new_n242), .b(new_n232), .c(new_n216), .d(new_n228), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n263), .o1(new_n277));
  aoai13aa1n12x5               g182(.a(new_n266), .b(new_n277), .c(new_n276), .d(new_n248), .o1(new_n278));
  oaih12aa1n02x5               g183(.a(new_n268), .b(new_n278), .c(new_n275), .o1(new_n279));
  aoi012aa1n03x5               g184(.a(new_n273), .b(new_n279), .c(new_n271), .o1(new_n280));
  nor002aa1n02x5               g185(.a(new_n280), .b(new_n274), .o1(\s[28] ));
  norb02aa1n02x5               g186(.a(new_n268), .b(new_n273), .out0(new_n282));
  aobi12aa1n02x7               g187(.a(new_n282), .b(new_n267), .c(new_n265), .out0(new_n283));
  oao003aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .c(new_n271), .carry(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[28] ), .b(\a[29] ), .out0(new_n285));
  nano22aa1n03x5               g190(.a(new_n283), .b(new_n284), .c(new_n285), .out0(new_n286));
  oaih12aa1n02x5               g191(.a(new_n282), .b(new_n278), .c(new_n275), .o1(new_n287));
  aoi012aa1n03x5               g192(.a(new_n285), .b(new_n287), .c(new_n284), .o1(new_n288));
  norp02aa1n03x5               g193(.a(new_n288), .b(new_n286), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n117), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g195(.a(new_n268), .b(new_n285), .c(new_n273), .out0(new_n291));
  aobi12aa1n03x7               g196(.a(new_n291), .b(new_n267), .c(new_n265), .out0(new_n292));
  oao003aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n284), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[29] ), .b(\a[30] ), .out0(new_n294));
  nano22aa1n03x5               g199(.a(new_n292), .b(new_n293), .c(new_n294), .out0(new_n295));
  tech160nm_fioai012aa1n04x5   g200(.a(new_n291), .b(new_n278), .c(new_n275), .o1(new_n296));
  tech160nm_fiaoi012aa1n02p5x5 g201(.a(new_n294), .b(new_n296), .c(new_n293), .o1(new_n297));
  norp02aa1n03x5               g202(.a(new_n297), .b(new_n295), .o1(\s[30] ));
  norb02aa1n02x5               g203(.a(new_n291), .b(new_n294), .out0(new_n299));
  aobi12aa1n03x5               g204(.a(new_n299), .b(new_n267), .c(new_n265), .out0(new_n300));
  oao003aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .c(new_n293), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  nano22aa1n03x5               g207(.a(new_n300), .b(new_n301), .c(new_n302), .out0(new_n303));
  oaih12aa1n02x5               g208(.a(new_n299), .b(new_n278), .c(new_n275), .o1(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n302), .b(new_n304), .c(new_n301), .o1(new_n305));
  norp02aa1n03x5               g210(.a(new_n305), .b(new_n303), .o1(\s[31] ));
  xnbna2aa1n03x5               g211(.a(new_n118), .b(new_n112), .c(new_n113), .out0(\s[3] ));
  oaoi03aa1n02x5               g212(.a(\a[3] ), .b(\b[2] ), .c(new_n118), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g214(.a(new_n158), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb03aa1n02x5               g215(.a(new_n108), .b(new_n119), .c(new_n156), .out0(new_n311));
  xobna2aa1n03x5               g216(.a(new_n103), .b(new_n311), .c(new_n105), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g217(.a(new_n123), .b(new_n158), .c(new_n107), .o(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g219(.a(new_n100), .b(new_n313), .c(new_n101), .o1(new_n315));
  xnrb03aa1n02x5               g220(.a(new_n315), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g221(.a(new_n125), .b(new_n120), .c(new_n124), .out0(\s[9] ));
endmodule


