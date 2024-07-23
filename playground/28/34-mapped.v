// Benchmark "adder" written by ABC on Thu Jul 18 02:36:57 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n321, new_n324,
    new_n327, new_n328, new_n329, new_n331, new_n333;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nanp02aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand22aa1n03x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  tech160nm_fioai012aa1n03p5x5 g004(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n100));
  nor002aa1n02x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nor022aa1n08x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nona23aa1n03x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  aoi012aa1n02x5               g010(.a(new_n103), .b(new_n101), .c(new_n104), .o1(new_n106));
  oaih12aa1n06x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  norp02aa1n04x5               g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  nand42aa1n08x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  norb02aa1n02x7               g014(.a(new_n109), .b(new_n108), .out0(new_n110));
  nand22aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  orn002aa1n12x5               g016(.a(\a[8] ), .b(\b[7] ), .o(new_n112));
  nor002aa1n04x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nand02aa1n03x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nor042aa1n04x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nona23aa1n02x4               g021(.a(new_n115), .b(new_n114), .c(new_n116), .d(new_n113), .out0(new_n117));
  nano32aa1n03x7               g022(.a(new_n117), .b(new_n110), .c(new_n111), .d(new_n112), .out0(new_n118));
  nanb03aa1n09x5               g023(.a(new_n108), .b(new_n115), .c(new_n109), .out0(new_n119));
  oai112aa1n06x5               g024(.a(new_n111), .b(new_n112), .c(new_n113), .d(new_n116), .o1(new_n120));
  aob012aa1n02x5               g025(.a(new_n112), .b(new_n108), .c(new_n111), .out0(new_n121));
  oabi12aa1n18x5               g026(.a(new_n121), .b(new_n120), .c(new_n119), .out0(new_n122));
  nor042aa1n02x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  tech160nm_finand02aa1n03p5x5 g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n123), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n122), .c(new_n107), .d(new_n118), .o1(new_n126));
  oai012aa1n02x5               g031(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand42aa1n20x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nor042aa1n06x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nand42aa1n08x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanb02aa1n02x5               g036(.a(new_n130), .b(new_n131), .out0(new_n132));
  oai022aa1d24x5               g037(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(new_n126), .b(new_n134), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n132), .b(new_n135), .c(new_n129), .out0(\s[11] ));
  inv000aa1d42x5               g041(.a(\b[11] ), .o1(new_n137));
  nanb03aa1n12x5               g042(.a(new_n130), .b(new_n131), .c(new_n129), .out0(new_n138));
  aoib12aa1n02x5               g043(.a(new_n130), .b(new_n135), .c(new_n138), .out0(new_n139));
  xorb03aa1n02x5               g044(.a(new_n139), .b(new_n137), .c(\a[12] ), .out0(\s[12] ));
  nor002aa1n02x5               g045(.a(\b[9] ), .b(\a[10] ), .o1(new_n141));
  nano23aa1n06x5               g046(.a(new_n141), .b(new_n123), .c(new_n124), .d(new_n129), .out0(new_n142));
  norp02aa1n04x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand42aa1n06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nano23aa1n06x5               g049(.a(new_n130), .b(new_n143), .c(new_n144), .d(new_n131), .out0(new_n145));
  nand22aa1n09x5               g050(.a(new_n145), .b(new_n142), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  aoai13aa1n06x5               g052(.a(new_n147), .b(new_n122), .c(new_n107), .d(new_n118), .o1(new_n148));
  nanb02aa1n03x5               g053(.a(\a[12] ), .b(new_n137), .out0(new_n149));
  nand23aa1n06x5               g054(.a(new_n133), .b(new_n149), .c(new_n144), .o1(new_n150));
  oai012aa1n09x5               g055(.a(new_n144), .b(new_n143), .c(new_n130), .o1(new_n151));
  oai012aa1d24x5               g056(.a(new_n151), .b(new_n150), .c(new_n138), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nor042aa1n06x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand42aa1n06x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nanb02aa1n02x5               g060(.a(new_n154), .b(new_n155), .out0(new_n156));
  xobna2aa1n03x5               g061(.a(new_n156), .b(new_n148), .c(new_n153), .out0(\s[13] ));
  inv020aa1n02x5               g062(.a(new_n154), .o1(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n156), .c(new_n148), .d(new_n153), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n03x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand42aa1n03x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nona23aa1n03x5               g067(.a(new_n162), .b(new_n155), .c(new_n154), .d(new_n161), .out0(new_n163));
  oaoi03aa1n12x5               g068(.a(\a[14] ), .b(\b[13] ), .c(new_n158), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  aoai13aa1n04x5               g070(.a(new_n165), .b(new_n163), .c(new_n148), .d(new_n153), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand42aa1n16x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nor002aa1n03x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand42aa1n06x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  aoai13aa1n03x5               g077(.a(new_n172), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n173));
  aoi112aa1n02x5               g078(.a(new_n172), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n173), .b(new_n174), .out0(\s[16] ));
  nano23aa1n06x5               g080(.a(new_n154), .b(new_n161), .c(new_n162), .d(new_n155), .out0(new_n176));
  nano23aa1n06x5               g081(.a(new_n168), .b(new_n170), .c(new_n171), .d(new_n169), .out0(new_n177));
  nano22aa1d15x5               g082(.a(new_n146), .b(new_n176), .c(new_n177), .out0(new_n178));
  aoai13aa1n12x5               g083(.a(new_n178), .b(new_n122), .c(new_n107), .d(new_n118), .o1(new_n179));
  aoai13aa1n06x5               g084(.a(new_n177), .b(new_n164), .c(new_n152), .d(new_n176), .o1(new_n180));
  oa0012aa1n02x5               g085(.a(new_n171), .b(new_n170), .c(new_n168), .o(new_n181));
  inv000aa1n02x5               g086(.a(new_n181), .o1(new_n182));
  nand23aa1d12x5               g087(.a(new_n179), .b(new_n180), .c(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nanp02aa1n02x5               g089(.a(\b[16] ), .b(\a[17] ), .o1(new_n185));
  nand02aa1d06x5               g090(.a(\b[17] ), .b(\a[18] ), .o1(new_n186));
  nor042aa1n06x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  nanb02aa1n02x5               g092(.a(new_n187), .b(new_n186), .out0(new_n188));
  nor042aa1n06x5               g093(.a(\b[16] ), .b(\a[17] ), .o1(new_n189));
  nona23aa1n02x4               g094(.a(new_n179), .b(new_n180), .c(new_n181), .d(new_n189), .out0(new_n190));
  xnbna2aa1n03x5               g095(.a(new_n188), .b(new_n190), .c(new_n185), .out0(\s[18] ));
  oaoi13aa1n06x5               g096(.a(new_n163), .b(new_n151), .c(new_n150), .d(new_n138), .o1(new_n192));
  oaoi13aa1n09x5               g097(.a(new_n181), .b(new_n177), .c(new_n192), .d(new_n164), .o1(new_n193));
  nano23aa1d15x5               g098(.a(new_n187), .b(new_n189), .c(new_n185), .d(new_n186), .out0(new_n194));
  inv000aa1d42x5               g099(.a(new_n194), .o1(new_n195));
  aoi012aa1n02x5               g100(.a(new_n187), .b(new_n189), .c(new_n186), .o1(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n195), .c(new_n193), .d(new_n179), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d24x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  inv000aa1d42x5               g106(.a(new_n196), .o1(new_n202));
  nanp02aa1n03x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nanb02aa1n09x5               g108(.a(new_n200), .b(new_n203), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  aoai13aa1n03x5               g110(.a(new_n205), .b(new_n202), .c(new_n183), .d(new_n194), .o1(new_n206));
  xnrc02aa1n12x5               g111(.a(\b[19] ), .b(\a[20] ), .out0(new_n207));
  tech160nm_fiaoi012aa1n02p5x5 g112(.a(new_n207), .b(new_n206), .c(new_n201), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n207), .o1(new_n209));
  aoi112aa1n02x5               g114(.a(new_n200), .b(new_n209), .c(new_n197), .d(new_n203), .o1(new_n210));
  norp02aa1n03x5               g115(.a(new_n208), .b(new_n210), .o1(\s[20] ));
  nona22aa1d24x5               g116(.a(new_n194), .b(new_n207), .c(new_n204), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  aoai13aa1n03x5               g118(.a(new_n203), .b(new_n187), .c(new_n189), .d(new_n186), .o1(new_n214));
  oab012aa1n02x4               g119(.a(new_n200), .b(\a[20] ), .c(\b[19] ), .out0(new_n215));
  aob012aa1n06x5               g120(.a(new_n213), .b(new_n214), .c(new_n215), .out0(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n212), .c(new_n193), .d(new_n179), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  inv040aa1n02x5               g124(.a(new_n219), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n212), .o1(new_n221));
  aoi022aa1n06x5               g126(.a(new_n214), .b(new_n215), .c(\b[19] ), .d(\a[20] ), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n219), .out0(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n222), .c(new_n183), .d(new_n221), .o1(new_n225));
  xnrc02aa1n12x5               g130(.a(\b[21] ), .b(\a[22] ), .out0(new_n226));
  aoi012aa1n03x5               g131(.a(new_n226), .b(new_n225), .c(new_n220), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n226), .o1(new_n228));
  aoi112aa1n02x5               g133(.a(new_n219), .b(new_n228), .c(new_n217), .d(new_n224), .o1(new_n229));
  norp02aa1n03x5               g134(.a(new_n227), .b(new_n229), .o1(\s[22] ));
  tech160nm_fixorc02aa1n03p5x5 g135(.a(\a[23] ), .b(\b[22] ), .out0(new_n231));
  nano22aa1d15x5               g136(.a(new_n226), .b(new_n220), .c(new_n223), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  nona22aa1n03x5               g138(.a(new_n183), .b(new_n212), .c(new_n233), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n231), .o1(new_n235));
  oaoi03aa1n03x5               g140(.a(\a[22] ), .b(\b[21] ), .c(new_n220), .o1(new_n236));
  aoi112aa1n03x5               g141(.a(new_n236), .b(new_n235), .c(new_n222), .d(new_n232), .o1(new_n237));
  tech160nm_finand02aa1n05x5   g142(.a(new_n234), .b(new_n237), .o1(new_n238));
  oabi12aa1n02x5               g143(.a(new_n236), .b(new_n216), .c(new_n233), .out0(new_n239));
  aoi013aa1n02x4               g144(.a(new_n239), .b(new_n183), .c(new_n221), .d(new_n232), .o1(new_n240));
  oai012aa1n02x5               g145(.a(new_n238), .b(new_n240), .c(new_n231), .o1(\s[23] ));
  and002aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .o(new_n242));
  xorc02aa1n02x5               g147(.a(\a[24] ), .b(\b[23] ), .out0(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n242), .c(new_n234), .d(new_n237), .o1(new_n244));
  nona22aa1n03x5               g149(.a(new_n238), .b(new_n243), .c(new_n242), .out0(new_n245));
  nanp02aa1n03x5               g150(.a(new_n245), .b(new_n244), .o1(\s[24] ));
  inv000aa1d42x5               g151(.a(\a[23] ), .o1(new_n247));
  inv040aa1d32x5               g152(.a(\a[24] ), .o1(new_n248));
  xroi22aa1d06x4               g153(.a(new_n247), .b(\b[22] ), .c(new_n248), .d(\b[23] ), .out0(new_n249));
  nano22aa1n06x5               g154(.a(new_n212), .b(new_n249), .c(new_n232), .out0(new_n250));
  inv030aa1n02x5               g155(.a(new_n250), .o1(new_n251));
  nanp02aa1n12x5               g156(.a(new_n249), .b(new_n232), .o1(new_n252));
  nanp02aa1n02x5               g157(.a(\b[23] ), .b(\a[24] ), .o1(new_n253));
  oai022aa1n02x5               g158(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n254));
  aoi022aa1n06x5               g159(.a(new_n249), .b(new_n236), .c(new_n253), .d(new_n254), .o1(new_n255));
  oai012aa1d24x5               g160(.a(new_n255), .b(new_n252), .c(new_n216), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  aoai13aa1n04x5               g162(.a(new_n257), .b(new_n251), .c(new_n193), .d(new_n179), .o1(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  xorc02aa1n02x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n256), .c(new_n183), .d(new_n250), .o1(new_n263));
  xorc02aa1n12x5               g168(.a(\a[26] ), .b(\b[25] ), .out0(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  tech160nm_fiaoi012aa1n02p5x5 g170(.a(new_n265), .b(new_n263), .c(new_n261), .o1(new_n266));
  aoi112aa1n02x5               g171(.a(new_n260), .b(new_n264), .c(new_n258), .d(new_n262), .o1(new_n267));
  nor002aa1n02x5               g172(.a(new_n266), .b(new_n267), .o1(\s[26] ));
  and002aa1n02x5               g173(.a(new_n264), .b(new_n262), .o(new_n269));
  inv000aa1n02x5               g174(.a(new_n269), .o1(new_n270));
  nor043aa1d12x5               g175(.a(new_n270), .b(new_n252), .c(new_n212), .o1(new_n271));
  inv020aa1n03x5               g176(.a(new_n271), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[26] ), .b(\b[25] ), .c(new_n261), .carry(new_n273));
  aobi12aa1n06x5               g178(.a(new_n273), .b(new_n256), .c(new_n269), .out0(new_n274));
  aoai13aa1n04x5               g179(.a(new_n274), .b(new_n272), .c(new_n193), .d(new_n179), .o1(new_n275));
  xorb03aa1n02x5               g180(.a(new_n275), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nanp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  xorc02aa1n02x5               g182(.a(\a[28] ), .b(\b[27] ), .out0(new_n278));
  nor042aa1n09x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  nanp03aa1n02x5               g184(.a(new_n222), .b(new_n232), .c(new_n249), .o1(new_n280));
  aoai13aa1n04x5               g185(.a(new_n273), .b(new_n270), .c(new_n280), .d(new_n255), .o1(new_n281));
  aoi112aa1n03x5               g186(.a(new_n279), .b(new_n281), .c(new_n183), .d(new_n271), .o1(new_n282));
  nano22aa1n03x5               g187(.a(new_n282), .b(new_n277), .c(new_n278), .out0(new_n283));
  nanp02aa1n02x5               g188(.a(new_n107), .b(new_n118), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n122), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n178), .o1(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n284), .c(new_n285), .o1(new_n287));
  nanp02aa1n02x5               g192(.a(new_n180), .b(new_n182), .o1(new_n288));
  oai012aa1n02x5               g193(.a(new_n271), .b(new_n288), .c(new_n287), .o1(new_n289));
  nona22aa1n02x5               g194(.a(new_n289), .b(new_n281), .c(new_n279), .out0(new_n290));
  aoi012aa1n02x5               g195(.a(new_n278), .b(new_n290), .c(new_n277), .o1(new_n291));
  norp02aa1n02x5               g196(.a(new_n291), .b(new_n283), .o1(\s[28] ));
  nano22aa1n02x4               g197(.a(new_n279), .b(new_n278), .c(new_n277), .out0(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n281), .c(new_n183), .d(new_n271), .o1(new_n294));
  inv000aa1d42x5               g199(.a(\a[28] ), .o1(new_n295));
  inv000aa1d42x5               g200(.a(\b[27] ), .o1(new_n296));
  oaoi03aa1n12x5               g201(.a(new_n295), .b(new_n296), .c(new_n279), .o1(new_n297));
  xorc02aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .out0(new_n298));
  inv000aa1n02x5               g203(.a(new_n298), .o1(new_n299));
  tech160nm_fiaoi012aa1n02p5x5 g204(.a(new_n299), .b(new_n294), .c(new_n297), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n297), .o1(new_n301));
  aoi112aa1n03x4               g206(.a(new_n298), .b(new_n301), .c(new_n275), .d(new_n293), .o1(new_n302));
  nor002aa1n02x5               g207(.a(new_n300), .b(new_n302), .o1(\s[29] ));
  xorb03aa1n02x5               g208(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g209(.a(new_n299), .b(new_n279), .c(new_n278), .d(new_n277), .out0(new_n305));
  aoai13aa1n03x5               g210(.a(new_n305), .b(new_n281), .c(new_n183), .d(new_n271), .o1(new_n306));
  oaoi03aa1n09x5               g211(.a(\a[29] ), .b(\b[28] ), .c(new_n297), .o1(new_n307));
  inv030aa1n03x5               g212(.a(new_n307), .o1(new_n308));
  xorc02aa1n12x5               g213(.a(\a[30] ), .b(\b[29] ), .out0(new_n309));
  inv000aa1d42x5               g214(.a(new_n309), .o1(new_n310));
  tech160nm_fiaoi012aa1n02p5x5 g215(.a(new_n310), .b(new_n306), .c(new_n308), .o1(new_n311));
  aoi112aa1n03x4               g216(.a(new_n309), .b(new_n307), .c(new_n275), .d(new_n305), .o1(new_n312));
  norp02aa1n03x5               g217(.a(new_n311), .b(new_n312), .o1(\s[30] ));
  and003aa1n02x5               g218(.a(new_n293), .b(new_n309), .c(new_n298), .o(new_n314));
  aoai13aa1n06x5               g219(.a(new_n314), .b(new_n281), .c(new_n183), .d(new_n271), .o1(new_n315));
  oaoi03aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .c(new_n308), .o1(new_n316));
  inv000aa1n02x5               g221(.a(new_n316), .o1(new_n317));
  xorc02aa1n02x5               g222(.a(\a[31] ), .b(\b[30] ), .out0(new_n318));
  inv000aa1d42x5               g223(.a(new_n318), .o1(new_n319));
  tech160nm_fiaoi012aa1n03p5x5 g224(.a(new_n319), .b(new_n315), .c(new_n317), .o1(new_n320));
  aoi112aa1n03x5               g225(.a(new_n318), .b(new_n316), .c(new_n275), .d(new_n314), .o1(new_n321));
  nor042aa1n03x5               g226(.a(new_n320), .b(new_n321), .o1(\s[31] ));
  xnrb03aa1n02x5               g227(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g228(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g230(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g231(.a(new_n116), .b(new_n115), .out0(new_n327));
  norb02aa1n02x5               g232(.a(new_n114), .b(new_n113), .out0(new_n328));
  oai112aa1n02x5               g233(.a(new_n106), .b(new_n328), .c(new_n105), .d(new_n100), .o1(new_n329));
  xnbna2aa1n03x5               g234(.a(new_n327), .b(new_n329), .c(new_n114), .out0(\s[6] ));
  aoi013aa1n02x4               g235(.a(new_n116), .b(new_n329), .c(new_n114), .d(new_n115), .o1(new_n331));
  xnrc02aa1n02x5               g236(.a(new_n331), .b(new_n110), .out0(\s[7] ));
  oaoi03aa1n02x5               g237(.a(\a[7] ), .b(\b[6] ), .c(new_n331), .o1(new_n333));
  xorb03aa1n02x5               g238(.a(new_n333), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g239(.a(new_n125), .b(new_n284), .c(new_n285), .out0(\s[9] ));
endmodule


