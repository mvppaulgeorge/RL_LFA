// Benchmark "adder" written by ABC on Wed Jul 17 21:01:07 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n337, new_n339,
    new_n340, new_n342, new_n343, new_n345, new_n346, new_n347, new_n348,
    new_n350;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  tech160nm_fixnrc02aa1n05x5   g003(.a(\b[2] ), .b(\a[3] ), .out0(new_n99));
  nand42aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand02aa1d16x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nor042aa1n06x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  oai012aa1n04x7               g007(.a(new_n100), .b(new_n102), .c(new_n101), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  orn002aa1n12x5               g009(.a(\a[4] ), .b(\b[3] ), .o(new_n105));
  nand42aa1n03x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nano22aa1n03x7               g011(.a(new_n104), .b(new_n105), .c(new_n106), .out0(new_n107));
  oai012aa1n09x5               g012(.a(new_n107), .b(new_n103), .c(new_n99), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nor042aa1n04x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand02aa1n12x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norb02aa1n03x5               g016(.a(new_n111), .b(new_n110), .out0(new_n112));
  aoi022aa1d24x5               g017(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\a[5] ), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\b[4] ), .o1(new_n115));
  aoi022aa1d24x5               g020(.a(new_n115), .b(new_n114), .c(\a[4] ), .d(\b[3] ), .o1(new_n116));
  nor002aa1n16x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  norp02aa1n06x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nona22aa1n02x4               g023(.a(new_n116), .b(new_n117), .c(new_n118), .out0(new_n119));
  nano32aa1n03x7               g024(.a(new_n119), .b(new_n112), .c(new_n113), .d(new_n109), .out0(new_n120));
  oai012aa1n02x5               g025(.a(new_n111), .b(new_n117), .c(new_n110), .o1(new_n121));
  nona23aa1n09x5               g026(.a(new_n113), .b(new_n111), .c(new_n117), .d(new_n110), .out0(new_n122));
  and002aa1n12x5               g027(.a(\b[5] ), .b(\a[6] ), .o(new_n123));
  aoi112aa1n09x5               g028(.a(new_n123), .b(new_n118), .c(new_n114), .d(new_n115), .o1(new_n124));
  oai012aa1n09x5               g029(.a(new_n121), .b(new_n122), .c(new_n124), .o1(new_n125));
  nand42aa1n03x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n97), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n125), .c(new_n120), .d(new_n108), .o1(new_n128));
  norp02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand42aa1n04x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  oaih22aa1d12x5               g036(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n132));
  nanb03aa1n02x5               g037(.a(new_n132), .b(new_n128), .c(new_n130), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n131), .c(new_n98), .d(new_n128), .o1(\s[10] ));
  nano23aa1n06x5               g039(.a(new_n97), .b(new_n129), .c(new_n130), .d(new_n126), .out0(new_n135));
  aoai13aa1n02x5               g040(.a(new_n135), .b(new_n125), .c(new_n120), .d(new_n108), .o1(new_n136));
  oai012aa1n02x5               g041(.a(new_n130), .b(new_n129), .c(new_n97), .o1(new_n137));
  xorc02aa1n02x5               g042(.a(\a[11] ), .b(\b[10] ), .out0(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n138), .b(new_n136), .c(new_n137), .out0(\s[11] ));
  inv040aa1d32x5               g044(.a(\a[11] ), .o1(new_n140));
  inv040aa1n08x5               g045(.a(\b[10] ), .o1(new_n141));
  nanp02aa1n04x5               g046(.a(new_n141), .b(new_n140), .o1(new_n142));
  xnrc02aa1n02x5               g047(.a(\b[10] ), .b(\a[11] ), .out0(new_n143));
  aoai13aa1n02x5               g048(.a(new_n142), .b(new_n143), .c(new_n136), .d(new_n137), .o1(new_n144));
  inv000aa1d42x5               g049(.a(\b[11] ), .o1(new_n145));
  nanb02aa1d24x5               g050(.a(\a[12] ), .b(new_n145), .out0(new_n146));
  nand42aa1n10x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(new_n146), .b(new_n147), .o1(new_n148));
  aoai13aa1n02x5               g053(.a(new_n147), .b(new_n143), .c(new_n136), .d(new_n137), .o1(new_n149));
  nanb03aa1n02x5               g054(.a(new_n149), .b(new_n142), .c(new_n146), .out0(new_n150));
  aob012aa1n02x5               g055(.a(new_n150), .b(new_n148), .c(new_n144), .out0(\s[12] ));
  nona22aa1n06x5               g056(.a(new_n135), .b(new_n143), .c(new_n148), .out0(new_n152));
  inv000aa1n06x5               g057(.a(new_n152), .o1(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n125), .c(new_n120), .d(new_n108), .o1(new_n154));
  nor002aa1n02x5               g059(.a(\b[11] ), .b(\a[12] ), .o1(new_n155));
  aoai13aa1n04x5               g060(.a(new_n147), .b(new_n155), .c(new_n140), .d(new_n141), .o1(new_n156));
  oai112aa1n06x5               g061(.a(new_n146), .b(new_n147), .c(new_n141), .d(new_n140), .o1(new_n157));
  nanp03aa1d12x5               g062(.a(new_n132), .b(new_n142), .c(new_n130), .o1(new_n158));
  oai012aa1n18x5               g063(.a(new_n156), .b(new_n158), .c(new_n157), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  nor002aa1d32x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nand02aa1n04x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  xnbna2aa1n03x5               g068(.a(new_n163), .b(new_n154), .c(new_n160), .out0(\s[13] ));
  inv000aa1d42x5               g069(.a(new_n161), .o1(new_n165));
  aob012aa1n02x5               g070(.a(new_n163), .b(new_n154), .c(new_n160), .out0(new_n166));
  nor022aa1n12x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nand02aa1n04x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  nona23aa1n02x4               g074(.a(new_n166), .b(new_n168), .c(new_n167), .d(new_n161), .out0(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n169), .c(new_n165), .d(new_n166), .o1(\s[14] ));
  nona23aa1d16x5               g076(.a(new_n168), .b(new_n162), .c(new_n161), .d(new_n167), .out0(new_n172));
  inv000aa1d42x5               g077(.a(new_n172), .o1(new_n173));
  oaoi03aa1n02x5               g078(.a(\a[14] ), .b(\b[13] ), .c(new_n165), .o1(new_n174));
  aoi012aa1n02x5               g079(.a(new_n174), .b(new_n159), .c(new_n173), .o1(new_n175));
  tech160nm_fioai012aa1n03p5x5 g080(.a(new_n175), .b(new_n154), .c(new_n172), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n03x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  inv000aa1n02x5               g083(.a(new_n178), .o1(new_n179));
  tech160nm_fixnrc02aa1n04x5   g084(.a(\b[14] ), .b(\a[15] ), .out0(new_n180));
  nanb02aa1n02x5               g085(.a(new_n180), .b(new_n176), .out0(new_n181));
  xnrc02aa1n12x5               g086(.a(\b[15] ), .b(\a[16] ), .out0(new_n182));
  inv000aa1d42x5               g087(.a(new_n182), .o1(new_n183));
  and002aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .o(new_n184));
  oai022aa1n02x5               g089(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n185));
  nona22aa1n02x5               g090(.a(new_n181), .b(new_n184), .c(new_n185), .out0(new_n186));
  aoai13aa1n02x5               g091(.a(new_n186), .b(new_n183), .c(new_n181), .d(new_n179), .o1(\s[16] ));
  nor042aa1n06x5               g092(.a(new_n182), .b(new_n180), .o1(new_n188));
  nano22aa1n06x5               g093(.a(new_n152), .b(new_n173), .c(new_n188), .out0(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n125), .c(new_n108), .d(new_n120), .o1(new_n190));
  oaoi13aa1n03x5               g095(.a(new_n172), .b(new_n156), .c(new_n158), .d(new_n157), .o1(new_n191));
  oaoi03aa1n12x5               g096(.a(\a[16] ), .b(\b[15] ), .c(new_n179), .o1(new_n192));
  oaoi13aa1n03x5               g097(.a(new_n192), .b(new_n188), .c(new_n191), .d(new_n174), .o1(new_n193));
  xorc02aa1n02x5               g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  xnbna2aa1n03x5               g099(.a(new_n194), .b(new_n193), .c(new_n190), .out0(\s[17] ));
  nor042aa1n06x5               g100(.a(\b[16] ), .b(\a[17] ), .o1(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  aoi012aa1n12x5               g102(.a(new_n125), .b(new_n120), .c(new_n108), .o1(new_n198));
  nor043aa1n06x5               g103(.a(new_n172), .b(new_n180), .c(new_n182), .o1(new_n199));
  nano22aa1n03x7               g104(.a(new_n198), .b(new_n153), .c(new_n199), .out0(new_n200));
  aoai13aa1n12x5               g105(.a(new_n188), .b(new_n174), .c(new_n159), .d(new_n173), .o1(new_n201));
  inv000aa1d42x5               g106(.a(new_n192), .o1(new_n202));
  nanp02aa1n09x5               g107(.a(new_n201), .b(new_n202), .o1(new_n203));
  oaih12aa1n02x5               g108(.a(new_n194), .b(new_n203), .c(new_n200), .o1(new_n204));
  xorc02aa1n02x5               g109(.a(\a[18] ), .b(\b[17] ), .out0(new_n205));
  nanp02aa1n12x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  oaih22aa1d12x5               g111(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n207));
  nanb03aa1n03x5               g112(.a(new_n207), .b(new_n204), .c(new_n206), .out0(new_n208));
  aoai13aa1n02x5               g113(.a(new_n208), .b(new_n205), .c(new_n204), .d(new_n197), .o1(\s[18] ));
  nona23aa1n06x5               g114(.a(new_n199), .b(new_n135), .c(new_n143), .d(new_n148), .out0(new_n210));
  oai112aa1n06x5               g115(.a(new_n201), .b(new_n202), .c(new_n198), .d(new_n210), .o1(new_n211));
  nand42aa1n03x5               g116(.a(\b[16] ), .b(\a[17] ), .o1(new_n212));
  norp02aa1n02x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  nano23aa1n06x5               g118(.a(new_n196), .b(new_n213), .c(new_n206), .d(new_n212), .out0(new_n214));
  oaoi03aa1n02x5               g119(.a(\a[18] ), .b(\b[17] ), .c(new_n197), .o1(new_n215));
  xorc02aa1n12x5               g120(.a(\a[19] ), .b(\b[18] ), .out0(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n215), .c(new_n211), .d(new_n214), .o1(new_n217));
  aoi112aa1n02x5               g122(.a(new_n216), .b(new_n215), .c(new_n211), .d(new_n214), .o1(new_n218));
  norb02aa1n03x4               g123(.a(new_n217), .b(new_n218), .out0(\s[19] ));
  xnrc02aa1n02x5               g124(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g125(.a(\a[19] ), .o1(new_n221));
  nanb02aa1n02x5               g126(.a(\b[18] ), .b(new_n221), .out0(new_n222));
  tech160nm_fixorc02aa1n04x5   g127(.a(\a[20] ), .b(\b[19] ), .out0(new_n223));
  oai022aa1n04x5               g128(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n224));
  aoi012aa1n02x5               g129(.a(new_n224), .b(\a[20] ), .c(\b[19] ), .o1(new_n225));
  tech160nm_finand02aa1n05x5   g130(.a(new_n217), .b(new_n225), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n226), .b(new_n223), .c(new_n217), .d(new_n222), .o1(\s[20] ));
  nanp03aa1d12x5               g132(.a(new_n214), .b(new_n216), .c(new_n223), .o1(new_n228));
  nand42aa1n02x5               g133(.a(\b[18] ), .b(\a[19] ), .o1(new_n229));
  nanp03aa1n02x5               g134(.a(new_n207), .b(new_n206), .c(new_n229), .o1(new_n230));
  aboi22aa1d24x5               g135(.a(new_n224), .b(new_n230), .c(\a[20] ), .d(\b[19] ), .out0(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n02x7               g137(.a(new_n232), .b(new_n228), .c(new_n193), .d(new_n190), .o1(new_n233));
  nor042aa1n04x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  nanp02aa1n02x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n235), .b(new_n234), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n228), .o1(new_n237));
  aoi112aa1n02x5               g142(.a(new_n236), .b(new_n231), .c(new_n211), .d(new_n237), .o1(new_n238));
  aoi012aa1n02x5               g143(.a(new_n238), .b(new_n233), .c(new_n236), .o1(\s[21] ));
  nor022aa1n12x5               g144(.a(\b[21] ), .b(\a[22] ), .o1(new_n240));
  nand02aa1n10x5               g145(.a(\b[21] ), .b(\a[22] ), .o1(new_n241));
  nanb02aa1n02x5               g146(.a(new_n240), .b(new_n241), .out0(new_n242));
  aoai13aa1n03x5               g147(.a(new_n242), .b(new_n234), .c(new_n233), .d(new_n235), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n236), .b(new_n231), .c(new_n211), .d(new_n237), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n241), .o1(new_n245));
  oai022aa1n02x5               g150(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n246));
  nona22aa1n03x5               g151(.a(new_n244), .b(new_n245), .c(new_n246), .out0(new_n247));
  nanp02aa1n03x5               g152(.a(new_n243), .b(new_n247), .o1(\s[22] ));
  nano23aa1n02x4               g153(.a(new_n234), .b(new_n240), .c(new_n241), .d(new_n235), .out0(new_n249));
  norb02aa1n02x5               g154(.a(new_n249), .b(new_n228), .out0(new_n250));
  oai012aa1n06x5               g155(.a(new_n250), .b(new_n203), .c(new_n200), .o1(new_n251));
  aoi013aa1n09x5               g156(.a(new_n224), .b(new_n207), .c(new_n229), .d(new_n206), .o1(new_n252));
  tech160nm_fiaoi012aa1n04x5   g157(.a(new_n234), .b(\a[20] ), .c(\b[19] ), .o1(new_n253));
  aoi012aa1n03x5               g158(.a(new_n240), .b(\a[21] ), .c(\b[20] ), .o1(new_n254));
  nand23aa1n06x5               g159(.a(new_n253), .b(new_n254), .c(new_n241), .o1(new_n255));
  oaih12aa1n12x5               g160(.a(new_n241), .b(new_n240), .c(new_n234), .o1(new_n256));
  oai012aa1d24x5               g161(.a(new_n256), .b(new_n252), .c(new_n255), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n251), .b(new_n258), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[23] ), .b(\b[22] ), .out0(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  oai112aa1n02x5               g166(.a(new_n256), .b(new_n261), .c(new_n252), .d(new_n255), .o1(new_n262));
  aboi22aa1n03x5               g167(.a(new_n262), .b(new_n251), .c(new_n259), .d(new_n260), .out0(\s[23] ));
  orn002aa1n02x5               g168(.a(\a[23] ), .b(\b[22] ), .o(new_n264));
  aoai13aa1n02x5               g169(.a(new_n260), .b(new_n257), .c(new_n211), .d(new_n250), .o1(new_n265));
  tech160nm_fixorc02aa1n04x5   g170(.a(\a[24] ), .b(\b[23] ), .out0(new_n266));
  nanp02aa1n02x5               g171(.a(\b[23] ), .b(\a[24] ), .o1(new_n267));
  oai022aa1n02x5               g172(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n268));
  norb02aa1n02x5               g173(.a(new_n267), .b(new_n268), .out0(new_n269));
  aoai13aa1n04x5               g174(.a(new_n269), .b(new_n261), .c(new_n251), .d(new_n258), .o1(new_n270));
  aoai13aa1n03x5               g175(.a(new_n270), .b(new_n266), .c(new_n265), .d(new_n264), .o1(\s[24] ));
  nano32aa1n03x7               g176(.a(new_n228), .b(new_n266), .c(new_n249), .d(new_n260), .out0(new_n272));
  nand22aa1n12x5               g177(.a(new_n266), .b(new_n260), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  aoi022aa1n12x5               g179(.a(new_n257), .b(new_n274), .c(new_n267), .d(new_n268), .o1(new_n275));
  inv030aa1n08x5               g180(.a(new_n275), .o1(new_n276));
  xorc02aa1n02x5               g181(.a(\a[25] ), .b(\b[24] ), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n276), .c(new_n211), .d(new_n272), .o1(new_n278));
  aoi112aa1n02x5               g183(.a(new_n276), .b(new_n277), .c(new_n211), .d(new_n272), .o1(new_n279));
  norb02aa1n03x4               g184(.a(new_n278), .b(new_n279), .out0(\s[25] ));
  nor042aa1n03x5               g185(.a(\b[24] ), .b(\a[25] ), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n281), .o1(new_n282));
  xorc02aa1n02x5               g187(.a(\a[26] ), .b(\b[25] ), .out0(new_n283));
  oai022aa1n02x5               g188(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n284));
  aoi012aa1n02x5               g189(.a(new_n284), .b(\a[26] ), .c(\b[25] ), .o1(new_n285));
  tech160nm_finand02aa1n05x5   g190(.a(new_n278), .b(new_n285), .o1(new_n286));
  aoai13aa1n03x5               g191(.a(new_n286), .b(new_n283), .c(new_n278), .d(new_n282), .o1(\s[26] ));
  and002aa1n02x5               g192(.a(new_n283), .b(new_n277), .o(new_n288));
  nand02aa1d04x5               g193(.a(new_n272), .b(new_n288), .o1(new_n289));
  oabi12aa1n18x5               g194(.a(new_n289), .b(new_n203), .c(new_n200), .out0(new_n290));
  oaoi13aa1n09x5               g195(.a(new_n273), .b(new_n256), .c(new_n252), .d(new_n255), .o1(new_n291));
  oaoi03aa1n02x5               g196(.a(\a[24] ), .b(\b[23] ), .c(new_n264), .o1(new_n292));
  oaoi03aa1n02x5               g197(.a(\a[26] ), .b(\b[25] ), .c(new_n282), .o1(new_n293));
  oaoi13aa1n09x5               g198(.a(new_n293), .b(new_n288), .c(new_n291), .d(new_n292), .o1(new_n294));
  aoai13aa1n06x5               g199(.a(new_n294), .b(new_n289), .c(new_n193), .d(new_n190), .o1(new_n295));
  xorc02aa1n12x5               g200(.a(\a[27] ), .b(\b[26] ), .out0(new_n296));
  aoi112aa1n03x5               g201(.a(new_n296), .b(new_n293), .c(new_n276), .d(new_n288), .o1(new_n297));
  aoi022aa1n02x5               g202(.a(new_n295), .b(new_n296), .c(new_n297), .d(new_n290), .o1(\s[27] ));
  norp02aa1n02x5               g203(.a(\b[26] ), .b(\a[27] ), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n299), .o1(new_n300));
  nanp02aa1n03x5               g205(.a(new_n295), .b(new_n296), .o1(new_n301));
  xorc02aa1n02x5               g206(.a(\a[28] ), .b(\b[27] ), .out0(new_n302));
  inv000aa1d42x5               g207(.a(new_n296), .o1(new_n303));
  oai022aa1n02x5               g208(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n304));
  aoi012aa1n02x5               g209(.a(new_n304), .b(\a[28] ), .c(\b[27] ), .o1(new_n305));
  aoai13aa1n04x5               g210(.a(new_n305), .b(new_n303), .c(new_n290), .d(new_n294), .o1(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n302), .c(new_n301), .d(new_n300), .o1(\s[28] ));
  and002aa1n06x5               g212(.a(new_n302), .b(new_n296), .o(new_n308));
  inv000aa1d42x5               g213(.a(new_n308), .o1(new_n309));
  inv000aa1d42x5               g214(.a(\b[27] ), .o1(new_n310));
  oaib12aa1n09x5               g215(.a(new_n304), .b(new_n310), .c(\a[28] ), .out0(new_n311));
  inv000aa1d42x5               g216(.a(new_n311), .o1(new_n312));
  tech160nm_fixorc02aa1n03p5x5 g217(.a(\a[29] ), .b(\b[28] ), .out0(new_n313));
  norb02aa1n02x5               g218(.a(new_n313), .b(new_n312), .out0(new_n314));
  aoai13aa1n02x7               g219(.a(new_n314), .b(new_n309), .c(new_n290), .d(new_n294), .o1(new_n315));
  inv000aa1d42x5               g220(.a(new_n313), .o1(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n312), .c(new_n295), .d(new_n308), .o1(new_n317));
  nanp02aa1n03x5               g222(.a(new_n317), .b(new_n315), .o1(\s[29] ));
  xorb03aa1n02x5               g223(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g224(.a(new_n316), .b(new_n296), .c(new_n302), .out0(new_n320));
  oaoi03aa1n02x5               g225(.a(\a[29] ), .b(\b[28] ), .c(new_n311), .o1(new_n321));
  xnrc02aa1n02x5               g226(.a(\b[29] ), .b(\a[30] ), .out0(new_n322));
  aoai13aa1n03x5               g227(.a(new_n322), .b(new_n321), .c(new_n295), .d(new_n320), .o1(new_n323));
  inv000aa1d42x5               g228(.a(new_n320), .o1(new_n324));
  norp02aa1n02x5               g229(.a(new_n321), .b(new_n322), .o1(new_n325));
  aoai13aa1n02x7               g230(.a(new_n325), .b(new_n324), .c(new_n290), .d(new_n294), .o1(new_n326));
  nanp02aa1n03x5               g231(.a(new_n323), .b(new_n326), .o1(\s[30] ));
  nano32aa1n03x7               g232(.a(new_n322), .b(new_n313), .c(new_n302), .d(new_n296), .out0(new_n328));
  aoi012aa1n02x5               g233(.a(new_n325), .b(\a[30] ), .c(\b[29] ), .o1(new_n329));
  xnrc02aa1n02x5               g234(.a(\b[30] ), .b(\a[31] ), .out0(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n329), .c(new_n295), .d(new_n328), .o1(new_n331));
  inv000aa1d42x5               g236(.a(new_n328), .o1(new_n332));
  norp02aa1n02x5               g237(.a(new_n329), .b(new_n330), .o1(new_n333));
  aoai13aa1n02x7               g238(.a(new_n333), .b(new_n332), .c(new_n290), .d(new_n294), .o1(new_n334));
  nanp02aa1n03x5               g239(.a(new_n331), .b(new_n334), .o1(\s[31] ));
  xnrb03aa1n02x5               g240(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oao003aa1n02x5               g241(.a(\a[3] ), .b(\b[2] ), .c(new_n103), .carry(new_n337));
  aoai13aa1n02x5               g242(.a(new_n108), .b(new_n337), .c(new_n106), .d(new_n105), .o1(\s[4] ));
  nanp02aa1n02x5               g243(.a(new_n115), .b(new_n114), .o1(new_n339));
  aoi022aa1n02x5               g244(.a(new_n108), .b(new_n106), .c(new_n339), .d(new_n109), .o1(new_n340));
  aoi013aa1n02x4               g245(.a(new_n340), .b(new_n116), .c(new_n109), .d(new_n108), .o1(\s[5] ));
  norp02aa1n02x5               g246(.a(new_n123), .b(new_n118), .o1(new_n342));
  nanp03aa1n02x5               g247(.a(new_n108), .b(new_n109), .c(new_n116), .o1(new_n343));
  xnbna2aa1n03x5               g248(.a(new_n342), .b(new_n343), .c(new_n339), .out0(\s[6] ));
  norb02aa1n02x5               g249(.a(new_n113), .b(new_n117), .out0(new_n345));
  aob012aa1n02x5               g250(.a(new_n345), .b(new_n343), .c(new_n124), .out0(new_n346));
  xnrc02aa1n02x5               g251(.a(\b[6] ), .b(\a[7] ), .out0(new_n347));
  aoai13aa1n02x5               g252(.a(new_n347), .b(new_n123), .c(new_n343), .d(new_n124), .o1(new_n348));
  and002aa1n02x5               g253(.a(new_n348), .b(new_n346), .o(\s[7] ));
  inv000aa1d42x5               g254(.a(new_n117), .o1(new_n350));
  xnbna2aa1n03x5               g255(.a(new_n112), .b(new_n346), .c(new_n350), .out0(\s[8] ));
  xnbna2aa1n03x5               g256(.a(new_n198), .b(new_n126), .c(new_n98), .out0(\s[9] ));
endmodule


