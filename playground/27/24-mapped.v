// Benchmark "adder" written by ABC on Thu Jul 18 02:00:05 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n169, new_n170,
    new_n171, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n207, new_n208, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n312, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n321, new_n322, new_n323, new_n324, new_n326,
    new_n327, new_n328, new_n329, new_n330, new_n331, new_n332, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n341, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n352,
    new_n355, new_n356, new_n358, new_n360;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n20x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1n16x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n12x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor002aa1n16x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  nand22aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand22aa1n04x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nor042aa1n03x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  oaih12aa1n12x5               g009(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n105));
  inv040aa1d32x5               g010(.a(\a[4] ), .o1(new_n106));
  inv030aa1d32x5               g011(.a(\b[3] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(new_n107), .b(new_n106), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  nand02aa1n03x5               g014(.a(new_n108), .b(new_n109), .o1(new_n110));
  inv040aa1d32x5               g015(.a(\a[3] ), .o1(new_n111));
  inv030aa1d32x5               g016(.a(\b[2] ), .o1(new_n112));
  nanp02aa1n12x5               g017(.a(new_n112), .b(new_n111), .o1(new_n113));
  nanp02aa1n04x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nanp02aa1n04x5               g019(.a(new_n113), .b(new_n114), .o1(new_n115));
  oaoi03aa1n06x5               g020(.a(\a[4] ), .b(\b[3] ), .c(new_n113), .o1(new_n116));
  inv030aa1n02x5               g021(.a(new_n116), .o1(new_n117));
  oai013aa1n09x5               g022(.a(new_n117), .b(new_n110), .c(new_n105), .d(new_n115), .o1(new_n118));
  nor022aa1n16x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  nand02aa1d06x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nor022aa1n16x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  nand02aa1n04x5               g026(.a(\b[6] ), .b(\a[7] ), .o1(new_n122));
  nona23aa1n09x5               g027(.a(new_n122), .b(new_n120), .c(new_n119), .d(new_n121), .out0(new_n123));
  xnrc02aa1n02x5               g028(.a(\b[5] ), .b(\a[6] ), .out0(new_n124));
  xnrc02aa1n03x5               g029(.a(\b[4] ), .b(\a[5] ), .out0(new_n125));
  norp03aa1d12x5               g030(.a(new_n123), .b(new_n124), .c(new_n125), .o1(new_n126));
  inv000aa1d42x5               g031(.a(\b[4] ), .o1(new_n127));
  nanb02aa1n12x5               g032(.a(\a[5] ), .b(new_n127), .out0(new_n128));
  oaoi03aa1n09x5               g033(.a(\a[6] ), .b(\b[5] ), .c(new_n128), .o1(new_n129));
  inv000aa1d42x5               g034(.a(\a[7] ), .o1(new_n130));
  inv000aa1d42x5               g035(.a(\b[6] ), .o1(new_n131));
  aoai13aa1n03x5               g036(.a(new_n120), .b(new_n119), .c(new_n130), .d(new_n131), .o1(new_n132));
  oaib12aa1n09x5               g037(.a(new_n132), .b(new_n123), .c(new_n129), .out0(new_n133));
  xorc02aa1n12x5               g038(.a(\a[9] ), .b(\b[8] ), .out0(new_n134));
  aoai13aa1n06x5               g039(.a(new_n134), .b(new_n133), .c(new_n118), .d(new_n126), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n99), .b(new_n135), .c(new_n101), .out0(\s[10] ));
  inv000aa1d42x5               g041(.a(new_n97), .o1(new_n137));
  inv000aa1d42x5               g042(.a(new_n99), .o1(new_n138));
  aoai13aa1n04x5               g043(.a(new_n137), .b(new_n138), .c(new_n135), .d(new_n101), .o1(new_n139));
  xorb03aa1n02x5               g044(.a(new_n139), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1n20x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand22aa1n04x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  nor002aa1n04x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanp02aa1n04x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  aoi112aa1n03x4               g051(.a(new_n146), .b(new_n141), .c(new_n139), .d(new_n143), .o1(new_n147));
  inv000aa1d42x5               g052(.a(new_n141), .o1(new_n148));
  norp03aa1d12x5               g053(.a(new_n105), .b(new_n110), .c(new_n115), .o1(new_n149));
  oai012aa1n12x5               g054(.a(new_n126), .b(new_n149), .c(new_n116), .o1(new_n150));
  nano23aa1n06x5               g055(.a(new_n119), .b(new_n121), .c(new_n122), .d(new_n120), .out0(new_n151));
  aobi12aa1n18x5               g056(.a(new_n132), .b(new_n151), .c(new_n129), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n134), .o1(new_n153));
  aoai13aa1n03x5               g058(.a(new_n101), .b(new_n153), .c(new_n150), .d(new_n152), .o1(new_n154));
  aoai13aa1n02x5               g059(.a(new_n143), .b(new_n97), .c(new_n154), .d(new_n98), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n146), .o1(new_n156));
  tech160nm_fiaoi012aa1n02p5x5 g061(.a(new_n156), .b(new_n155), .c(new_n148), .o1(new_n157));
  nor002aa1n02x5               g062(.a(new_n157), .b(new_n147), .o1(\s[12] ));
  nona23aa1n09x5               g063(.a(new_n145), .b(new_n142), .c(new_n141), .d(new_n144), .out0(new_n159));
  nano22aa1n12x5               g064(.a(new_n159), .b(new_n134), .c(new_n99), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  nano23aa1n02x5               g066(.a(new_n141), .b(new_n144), .c(new_n145), .d(new_n142), .out0(new_n162));
  aoi012aa1d18x5               g067(.a(new_n97), .b(new_n100), .c(new_n98), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  oai012aa1n02x7               g069(.a(new_n145), .b(new_n144), .c(new_n141), .o1(new_n165));
  aobi12aa1n03x5               g070(.a(new_n165), .b(new_n162), .c(new_n164), .out0(new_n166));
  aoai13aa1n06x5               g071(.a(new_n166), .b(new_n161), .c(new_n150), .d(new_n152), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n16x5               g073(.a(\b[12] ), .b(\a[13] ), .o1(new_n169));
  nand42aa1n02x5               g074(.a(\b[12] ), .b(\a[13] ), .o1(new_n170));
  aoi012aa1n02x5               g075(.a(new_n169), .b(new_n167), .c(new_n170), .o1(new_n171));
  xnrb03aa1n03x5               g076(.a(new_n171), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  aoai13aa1n06x5               g077(.a(new_n160), .b(new_n133), .c(new_n118), .d(new_n126), .o1(new_n173));
  nor042aa1n06x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nand22aa1n12x5               g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  nona23aa1n03x5               g080(.a(new_n175), .b(new_n170), .c(new_n169), .d(new_n174), .out0(new_n176));
  aoi012aa1d18x5               g081(.a(new_n174), .b(new_n169), .c(new_n175), .o1(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n176), .c(new_n173), .d(new_n166), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1d18x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nand42aa1n08x5               g085(.a(\b[14] ), .b(\a[15] ), .o1(new_n181));
  nanb02aa1d24x5               g086(.a(new_n180), .b(new_n181), .out0(new_n182));
  inv000aa1d42x5               g087(.a(new_n182), .o1(new_n183));
  nor042aa1n09x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nand02aa1n06x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  nanb02aa1n18x5               g090(.a(new_n184), .b(new_n185), .out0(new_n186));
  inv000aa1d42x5               g091(.a(new_n186), .o1(new_n187));
  aoi112aa1n02x5               g092(.a(new_n187), .b(new_n180), .c(new_n178), .d(new_n183), .o1(new_n188));
  inv000aa1d42x5               g093(.a(new_n180), .o1(new_n189));
  nano23aa1n02x5               g094(.a(new_n169), .b(new_n174), .c(new_n175), .d(new_n170), .out0(new_n190));
  inv000aa1d42x5               g095(.a(new_n177), .o1(new_n191));
  aoai13aa1n02x5               g096(.a(new_n183), .b(new_n191), .c(new_n167), .d(new_n190), .o1(new_n192));
  aoi012aa1n03x5               g097(.a(new_n186), .b(new_n192), .c(new_n189), .o1(new_n193));
  norp02aa1n02x5               g098(.a(new_n193), .b(new_n188), .o1(\s[16] ));
  nona23aa1n09x5               g099(.a(new_n185), .b(new_n181), .c(new_n180), .d(new_n184), .out0(new_n195));
  nor042aa1n02x5               g100(.a(new_n195), .b(new_n176), .o1(new_n196));
  nand02aa1n03x5               g101(.a(new_n160), .b(new_n196), .o1(new_n197));
  tech160nm_fioai012aa1n05x5   g102(.a(new_n165), .b(new_n159), .c(new_n163), .o1(new_n198));
  oai012aa1n02x5               g103(.a(new_n185), .b(new_n184), .c(new_n180), .o1(new_n199));
  tech160nm_fioai012aa1n04x5   g104(.a(new_n199), .b(new_n195), .c(new_n177), .o1(new_n200));
  aoi012aa1d18x5               g105(.a(new_n200), .b(new_n198), .c(new_n196), .o1(new_n201));
  aoai13aa1n12x5               g106(.a(new_n201), .b(new_n197), .c(new_n150), .d(new_n152), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  norp02aa1n02x5               g108(.a(\b[16] ), .b(\a[17] ), .o1(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  xorc02aa1n02x5               g110(.a(\a[18] ), .b(\b[17] ), .out0(new_n206));
  inv000aa1d42x5               g111(.a(\a[17] ), .o1(new_n207));
  oaib12aa1n06x5               g112(.a(new_n202), .b(new_n207), .c(\b[16] ), .out0(new_n208));
  xnbna2aa1n03x5               g113(.a(new_n206), .b(new_n208), .c(new_n205), .out0(\s[18] ));
  nona22aa1n03x5               g114(.a(new_n190), .b(new_n186), .c(new_n182), .out0(new_n210));
  nano32aa1n03x7               g115(.a(new_n210), .b(new_n162), .c(new_n134), .d(new_n99), .out0(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n133), .c(new_n118), .d(new_n126), .o1(new_n212));
  inv020aa1n04x5               g117(.a(\a[18] ), .o1(new_n213));
  xroi22aa1d06x4               g118(.a(new_n207), .b(\b[16] ), .c(new_n213), .d(\b[17] ), .out0(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  nor042aa1n02x5               g120(.a(\b[17] ), .b(\a[18] ), .o1(new_n216));
  aoi112aa1n09x5               g121(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n217));
  nor042aa1n09x5               g122(.a(new_n217), .b(new_n216), .o1(new_n218));
  aoai13aa1n04x5               g123(.a(new_n218), .b(new_n215), .c(new_n212), .d(new_n201), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  nand02aa1n04x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  nor022aa1n16x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nanp02aa1n12x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  norb02aa1n12x5               g130(.a(new_n225), .b(new_n224), .out0(new_n226));
  aoi112aa1n02x5               g131(.a(new_n222), .b(new_n226), .c(new_n219), .d(new_n223), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n222), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n218), .o1(new_n229));
  norb02aa1n02x5               g134(.a(new_n223), .b(new_n222), .out0(new_n230));
  aoai13aa1n03x5               g135(.a(new_n230), .b(new_n229), .c(new_n202), .d(new_n214), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n226), .o1(new_n232));
  tech160nm_fiaoi012aa1n02p5x5 g137(.a(new_n232), .b(new_n231), .c(new_n228), .o1(new_n233));
  norp02aa1n03x5               g138(.a(new_n233), .b(new_n227), .o1(\s[20] ));
  nona23aa1d18x5               g139(.a(new_n225), .b(new_n223), .c(new_n222), .d(new_n224), .out0(new_n235));
  norb02aa1n02x5               g140(.a(new_n214), .b(new_n235), .out0(new_n236));
  inv020aa1n02x5               g141(.a(new_n236), .o1(new_n237));
  oaih12aa1n06x5               g142(.a(new_n225), .b(new_n224), .c(new_n222), .o1(new_n238));
  oai012aa1d24x5               g143(.a(new_n238), .b(new_n235), .c(new_n218), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n237), .c(new_n212), .d(new_n201), .o1(new_n241));
  xorb03aa1n02x5               g146(.a(new_n241), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1d32x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  xnrc02aa1n12x5               g148(.a(\b[20] ), .b(\a[21] ), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  xnrc02aa1n12x5               g150(.a(\b[21] ), .b(\a[22] ), .out0(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n243), .b(new_n247), .c(new_n241), .d(new_n245), .o1(new_n248));
  inv020aa1n02x5               g153(.a(new_n243), .o1(new_n249));
  aoai13aa1n03x5               g154(.a(new_n245), .b(new_n239), .c(new_n202), .d(new_n236), .o1(new_n250));
  aoi012aa1n03x5               g155(.a(new_n246), .b(new_n250), .c(new_n249), .o1(new_n251));
  norp02aa1n03x5               g156(.a(new_n251), .b(new_n248), .o1(\s[22] ));
  inv000aa1d42x5               g157(.a(new_n235), .o1(new_n253));
  nor042aa1n06x5               g158(.a(new_n246), .b(new_n244), .o1(new_n254));
  nand23aa1d12x5               g159(.a(new_n214), .b(new_n253), .c(new_n254), .o1(new_n255));
  oaoi03aa1n06x5               g160(.a(\a[22] ), .b(\b[21] ), .c(new_n249), .o1(new_n256));
  aoi012aa1d18x5               g161(.a(new_n256), .b(new_n239), .c(new_n254), .o1(new_n257));
  aoai13aa1n02x7               g162(.a(new_n257), .b(new_n255), .c(new_n212), .d(new_n201), .o1(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1n10x5               g164(.a(\b[22] ), .b(\a[23] ), .o1(new_n260));
  nand42aa1n02x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  norb02aa1n02x5               g166(.a(new_n261), .b(new_n260), .out0(new_n262));
  nor002aa1n03x5               g167(.a(\b[23] ), .b(\a[24] ), .o1(new_n263));
  nand42aa1n02x5               g168(.a(\b[23] ), .b(\a[24] ), .o1(new_n264));
  norb02aa1n02x5               g169(.a(new_n264), .b(new_n263), .out0(new_n265));
  aoi112aa1n03x4               g170(.a(new_n260), .b(new_n265), .c(new_n258), .d(new_n262), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n260), .o1(new_n267));
  inv030aa1n02x5               g172(.a(new_n255), .o1(new_n268));
  inv000aa1n02x5               g173(.a(new_n257), .o1(new_n269));
  aoai13aa1n03x5               g174(.a(new_n262), .b(new_n269), .c(new_n202), .d(new_n268), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n265), .o1(new_n271));
  aoi012aa1n06x5               g176(.a(new_n271), .b(new_n270), .c(new_n267), .o1(new_n272));
  nor042aa1n03x5               g177(.a(new_n272), .b(new_n266), .o1(\s[24] ));
  nona23aa1n12x5               g178(.a(new_n264), .b(new_n261), .c(new_n260), .d(new_n263), .out0(new_n274));
  nano32aa1n03x7               g179(.a(new_n274), .b(new_n214), .c(new_n254), .d(new_n253), .out0(new_n275));
  inv020aa1n02x5               g180(.a(new_n275), .o1(new_n276));
  inv020aa1n04x5               g181(.a(new_n274), .o1(new_n277));
  aoi012aa1n02x5               g182(.a(new_n263), .b(new_n260), .c(new_n264), .o1(new_n278));
  oaib12aa1n09x5               g183(.a(new_n278), .b(new_n274), .c(new_n256), .out0(new_n279));
  aoi013aa1n03x5               g184(.a(new_n279), .b(new_n239), .c(new_n254), .d(new_n277), .o1(new_n280));
  aoai13aa1n04x5               g185(.a(new_n280), .b(new_n276), .c(new_n212), .d(new_n201), .o1(new_n281));
  xorb03aa1n02x5               g186(.a(new_n281), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g187(.a(\b[24] ), .b(\a[25] ), .o1(new_n283));
  xorc02aa1n06x5               g188(.a(\a[25] ), .b(\b[24] ), .out0(new_n284));
  xorc02aa1n12x5               g189(.a(\a[26] ), .b(\b[25] ), .out0(new_n285));
  aoi112aa1n03x4               g190(.a(new_n283), .b(new_n285), .c(new_n281), .d(new_n284), .o1(new_n286));
  inv000aa1n03x5               g191(.a(new_n283), .o1(new_n287));
  inv030aa1n02x5               g192(.a(new_n280), .o1(new_n288));
  aoai13aa1n03x5               g193(.a(new_n284), .b(new_n288), .c(new_n202), .d(new_n275), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n285), .o1(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n290), .b(new_n289), .c(new_n287), .o1(new_n291));
  nor002aa1n02x5               g196(.a(new_n291), .b(new_n286), .o1(\s[26] ));
  xorc02aa1n02x5               g197(.a(\a[4] ), .b(\b[3] ), .out0(new_n293));
  norp02aa1n02x5               g198(.a(\b[2] ), .b(\a[3] ), .o1(new_n294));
  norb02aa1n02x5               g199(.a(new_n114), .b(new_n294), .out0(new_n295));
  nanb03aa1n02x5               g200(.a(new_n105), .b(new_n293), .c(new_n295), .out0(new_n296));
  xorc02aa1n02x5               g201(.a(\a[6] ), .b(\b[5] ), .out0(new_n297));
  xorc02aa1n02x5               g202(.a(\a[5] ), .b(\b[4] ), .out0(new_n298));
  nanp03aa1n02x5               g203(.a(new_n151), .b(new_n297), .c(new_n298), .o1(new_n299));
  aoai13aa1n06x5               g204(.a(new_n152), .b(new_n299), .c(new_n296), .d(new_n117), .o1(new_n300));
  oabi12aa1n03x5               g205(.a(new_n200), .b(new_n166), .c(new_n210), .out0(new_n301));
  and002aa1n06x5               g206(.a(new_n285), .b(new_n284), .o(new_n302));
  nano22aa1d15x5               g207(.a(new_n255), .b(new_n302), .c(new_n277), .out0(new_n303));
  aoai13aa1n06x5               g208(.a(new_n303), .b(new_n301), .c(new_n300), .d(new_n211), .o1(new_n304));
  oai112aa1n02x5               g209(.a(new_n230), .b(new_n226), .c(new_n217), .d(new_n216), .o1(new_n305));
  nanp02aa1n02x5               g210(.a(new_n247), .b(new_n245), .o1(new_n306));
  aoi112aa1n03x4               g211(.a(new_n306), .b(new_n274), .c(new_n305), .d(new_n238), .o1(new_n307));
  oaoi03aa1n09x5               g212(.a(\a[26] ), .b(\b[25] ), .c(new_n287), .o1(new_n308));
  oaoi13aa1n09x5               g213(.a(new_n308), .b(new_n302), .c(new_n307), .d(new_n279), .o1(new_n309));
  nor042aa1n03x5               g214(.a(\b[26] ), .b(\a[27] ), .o1(new_n310));
  nanp02aa1n02x5               g215(.a(\b[26] ), .b(\a[27] ), .o1(new_n311));
  norb02aa1n02x5               g216(.a(new_n311), .b(new_n310), .out0(new_n312));
  xnbna2aa1n03x5               g217(.a(new_n312), .b(new_n304), .c(new_n309), .out0(\s[27] ));
  inv000aa1d42x5               g218(.a(new_n310), .o1(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[27] ), .b(\a[28] ), .out0(new_n315));
  nona32aa1n03x5               g220(.a(new_n239), .b(new_n274), .c(new_n246), .d(new_n244), .out0(new_n316));
  aobi12aa1n02x5               g221(.a(new_n278), .b(new_n277), .c(new_n256), .out0(new_n317));
  inv000aa1d42x5               g222(.a(new_n302), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n308), .o1(new_n319));
  aoai13aa1n04x5               g224(.a(new_n319), .b(new_n318), .c(new_n316), .d(new_n317), .o1(new_n320));
  aoai13aa1n03x5               g225(.a(new_n311), .b(new_n320), .c(new_n202), .d(new_n303), .o1(new_n321));
  tech160nm_fiaoi012aa1n02p5x5 g226(.a(new_n315), .b(new_n321), .c(new_n314), .o1(new_n322));
  aoi022aa1n02x7               g227(.a(new_n304), .b(new_n309), .c(\b[26] ), .d(\a[27] ), .o1(new_n323));
  nano22aa1n03x5               g228(.a(new_n323), .b(new_n314), .c(new_n315), .out0(new_n324));
  norp02aa1n03x5               g229(.a(new_n322), .b(new_n324), .o1(\s[28] ));
  nano22aa1n02x4               g230(.a(new_n315), .b(new_n314), .c(new_n311), .out0(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n320), .c(new_n202), .d(new_n303), .o1(new_n327));
  oao003aa1n02x5               g232(.a(\a[28] ), .b(\b[27] ), .c(new_n314), .carry(new_n328));
  xnrc02aa1n02x5               g233(.a(\b[28] ), .b(\a[29] ), .out0(new_n329));
  tech160nm_fiaoi012aa1n02p5x5 g234(.a(new_n329), .b(new_n327), .c(new_n328), .o1(new_n330));
  aobi12aa1n02x7               g235(.a(new_n326), .b(new_n304), .c(new_n309), .out0(new_n331));
  nano22aa1n03x5               g236(.a(new_n331), .b(new_n328), .c(new_n329), .out0(new_n332));
  norp02aa1n03x5               g237(.a(new_n330), .b(new_n332), .o1(\s[29] ));
  xorb03aa1n02x5               g238(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g239(.a(new_n312), .b(new_n329), .c(new_n315), .out0(new_n335));
  aoai13aa1n03x5               g240(.a(new_n335), .b(new_n320), .c(new_n202), .d(new_n303), .o1(new_n336));
  oao003aa1n02x5               g241(.a(\a[29] ), .b(\b[28] ), .c(new_n328), .carry(new_n337));
  xnrc02aa1n02x5               g242(.a(\b[29] ), .b(\a[30] ), .out0(new_n338));
  tech160nm_fiaoi012aa1n02p5x5 g243(.a(new_n338), .b(new_n336), .c(new_n337), .o1(new_n339));
  aobi12aa1n02x7               g244(.a(new_n335), .b(new_n304), .c(new_n309), .out0(new_n340));
  nano22aa1n03x5               g245(.a(new_n340), .b(new_n337), .c(new_n338), .out0(new_n341));
  norp02aa1n03x5               g246(.a(new_n339), .b(new_n341), .o1(\s[30] ));
  norb03aa1n02x5               g247(.a(new_n326), .b(new_n338), .c(new_n329), .out0(new_n343));
  aobi12aa1n02x7               g248(.a(new_n343), .b(new_n304), .c(new_n309), .out0(new_n344));
  oao003aa1n02x5               g249(.a(\a[30] ), .b(\b[29] ), .c(new_n337), .carry(new_n345));
  xnrc02aa1n02x5               g250(.a(\b[30] ), .b(\a[31] ), .out0(new_n346));
  nano22aa1n03x5               g251(.a(new_n344), .b(new_n345), .c(new_n346), .out0(new_n347));
  aoai13aa1n03x5               g252(.a(new_n343), .b(new_n320), .c(new_n202), .d(new_n303), .o1(new_n348));
  tech160nm_fiaoi012aa1n02p5x5 g253(.a(new_n346), .b(new_n348), .c(new_n345), .o1(new_n349));
  norp02aa1n03x5               g254(.a(new_n349), .b(new_n347), .o1(\s[31] ));
  xnbna2aa1n03x5               g255(.a(new_n105), .b(new_n114), .c(new_n113), .out0(\s[3] ));
  oaoi03aa1n02x5               g256(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n352));
  xorb03aa1n02x5               g257(.a(new_n352), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g258(.a(new_n118), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g259(.a(\b[4] ), .b(\a[5] ), .o1(new_n355));
  oai013aa1n03x5               g260(.a(new_n355), .b(new_n149), .c(new_n116), .d(new_n125), .o1(new_n356));
  xnrc02aa1n02x5               g261(.a(new_n356), .b(new_n297), .out0(\s[6] ));
  oaoi03aa1n03x5               g262(.a(\a[6] ), .b(\b[5] ), .c(new_n356), .o1(new_n358));
  xorb03aa1n02x5               g263(.a(new_n358), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g264(.a(new_n130), .b(new_n131), .c(new_n358), .o1(new_n360));
  xnrb03aa1n03x5               g265(.a(new_n360), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g266(.a(new_n134), .b(new_n150), .c(new_n152), .out0(\s[9] ));
endmodule


