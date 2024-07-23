// Benchmark "adder" written by ABC on Wed Jul 17 23:38:02 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n299, new_n300, new_n301, new_n303, new_n305,
    new_n307, new_n309;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  inv000aa1n02x5               g002(.a(new_n97), .o1(new_n98));
  nand02aa1d12x5               g003(.a(\b[9] ), .b(\a[10] ), .o1(new_n99));
  inv040aa1d32x5               g004(.a(\a[3] ), .o1(new_n100));
  inv040aa1d32x5               g005(.a(\b[2] ), .o1(new_n101));
  nanp02aa1n04x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n02x5               g008(.a(new_n102), .b(new_n103), .o1(new_n104));
  nand42aa1n03x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nand22aa1n09x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  nor042aa1n03x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  oai012aa1n06x5               g012(.a(new_n105), .b(new_n107), .c(new_n106), .o1(new_n108));
  oa0022aa1n02x5               g013(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n109));
  oai012aa1n09x5               g014(.a(new_n109), .b(new_n108), .c(new_n104), .o1(new_n110));
  tech160nm_finand02aa1n03p5x5 g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  oai112aa1n02x5               g017(.a(new_n111), .b(new_n112), .c(\b[5] ), .d(\a[6] ), .o1(new_n113));
  nand02aa1n10x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\a[8] ), .o1(new_n115));
  nanb02aa1n03x5               g020(.a(\b[7] ), .b(new_n115), .out0(new_n116));
  oai112aa1n02x5               g021(.a(new_n116), .b(new_n114), .c(\b[6] ), .d(\a[7] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nanp02aa1n04x5               g023(.a(\b[3] ), .b(\a[4] ), .o1(new_n119));
  oai112aa1n02x5               g024(.a(new_n118), .b(new_n119), .c(\b[4] ), .d(\a[5] ), .o1(new_n120));
  nor043aa1n03x5               g025(.a(new_n117), .b(new_n120), .c(new_n113), .o1(new_n121));
  inv000aa1d42x5               g026(.a(new_n114), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\a[7] ), .o1(new_n123));
  inv000aa1d42x5               g028(.a(\b[6] ), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(new_n124), .b(new_n123), .o1(new_n125));
  nor002aa1n02x5               g030(.a(\b[5] ), .b(\a[6] ), .o1(new_n126));
  nor042aa1n04x5               g031(.a(\b[4] ), .b(\a[5] ), .o1(new_n127));
  aoi022aa1d24x5               g032(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n128));
  oaih12aa1n02x5               g033(.a(new_n128), .b(new_n127), .c(new_n126), .o1(new_n129));
  aoi013aa1n02x4               g034(.a(new_n122), .b(new_n129), .c(new_n125), .d(new_n116), .o1(new_n130));
  aoi012aa1n12x5               g035(.a(new_n130), .b(new_n121), .c(new_n110), .o1(new_n131));
  oaoi03aa1n02x5               g036(.a(\a[9] ), .b(\b[8] ), .c(new_n131), .o1(new_n132));
  xobna2aa1n03x5               g037(.a(new_n132), .b(new_n99), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g038(.a(new_n99), .o1(new_n134));
  nor042aa1n02x5               g039(.a(\b[8] ), .b(\a[9] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[8] ), .b(\a[9] ), .o1(new_n136));
  nona22aa1n02x4               g041(.a(new_n136), .b(new_n131), .c(new_n135), .out0(new_n137));
  nona32aa1n02x4               g042(.a(new_n137), .b(new_n135), .c(new_n134), .d(new_n97), .out0(new_n138));
  orn002aa1n02x5               g043(.a(\a[11] ), .b(\b[10] ), .o(new_n139));
  nanp02aa1n02x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(new_n139), .b(new_n140), .o1(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n138), .c(new_n99), .out0(\s[11] ));
  nona22aa1n02x4               g047(.a(new_n138), .b(new_n141), .c(new_n134), .out0(new_n143));
  xnrc02aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .out0(new_n144));
  aoi012aa1n02x5               g049(.a(new_n144), .b(new_n143), .c(new_n139), .o1(new_n145));
  nanp03aa1n02x5               g050(.a(new_n143), .b(new_n139), .c(new_n144), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n146), .b(new_n145), .out0(\s[12] ));
  oai112aa1n02x5               g052(.a(new_n98), .b(new_n99), .c(\b[8] ), .d(\a[9] ), .o1(new_n148));
  nanp03aa1n02x5               g053(.a(new_n139), .b(new_n136), .c(new_n140), .o1(new_n149));
  orn003aa1n02x5               g054(.a(new_n148), .b(new_n149), .c(new_n144), .o(new_n150));
  oai112aa1n03x5               g055(.a(new_n140), .b(new_n99), .c(new_n135), .d(new_n97), .o1(new_n151));
  oa0022aa1n02x5               g056(.a(\a[12] ), .b(\b[11] ), .c(\a[11] ), .d(\b[10] ), .o(new_n152));
  aoi022aa1n06x5               g057(.a(new_n151), .b(new_n152), .c(\b[11] ), .d(\a[12] ), .o1(new_n153));
  oabi12aa1n06x5               g058(.a(new_n153), .b(new_n131), .c(new_n150), .out0(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand22aa1n04x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n156), .b(new_n154), .c(new_n157), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d24x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand02aa1n08x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nona23aa1d16x5               g066(.a(new_n161), .b(new_n157), .c(new_n156), .d(new_n160), .out0(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  oai012aa1n12x5               g068(.a(new_n161), .b(new_n160), .c(new_n156), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  xnrc02aa1n12x5               g070(.a(\b[14] ), .b(\a[15] ), .out0(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  aoai13aa1n06x5               g072(.a(new_n167), .b(new_n165), .c(new_n154), .d(new_n163), .o1(new_n168));
  aoi112aa1n02x5               g073(.a(new_n167), .b(new_n165), .c(new_n154), .d(new_n163), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(\s[15] ));
  inv000aa1d42x5               g075(.a(\a[15] ), .o1(new_n171));
  inv000aa1d42x5               g076(.a(\b[14] ), .o1(new_n172));
  nanp02aa1n02x5               g077(.a(new_n172), .b(new_n171), .o1(new_n173));
  xnrc02aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .out0(new_n174));
  xobna2aa1n03x5               g079(.a(new_n174), .b(new_n168), .c(new_n173), .out0(\s[16] ));
  nor043aa1n02x5               g080(.a(new_n148), .b(new_n149), .c(new_n144), .o1(new_n176));
  nor043aa1n03x5               g081(.a(new_n162), .b(new_n166), .c(new_n174), .o1(new_n177));
  nand02aa1d04x5               g082(.a(new_n177), .b(new_n176), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\a[16] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\b[15] ), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n180), .b(new_n179), .o1(new_n181));
  oai022aa1n02x5               g086(.a(new_n171), .b(new_n172), .c(new_n180), .d(new_n179), .o1(new_n182));
  aoai13aa1n03x5               g087(.a(new_n181), .b(new_n182), .c(new_n164), .d(new_n173), .o1(new_n183));
  aoi012aa1n06x5               g088(.a(new_n183), .b(new_n177), .c(new_n153), .o1(new_n184));
  oai012aa1n18x5               g089(.a(new_n184), .b(new_n131), .c(new_n178), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g091(.a(\a[18] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\a[17] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[16] ), .o1(new_n189));
  oaoi03aa1n02x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  xroi22aa1d04x5               g096(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n192));
  nanp02aa1n02x5               g097(.a(new_n189), .b(new_n188), .o1(new_n193));
  oaoi03aa1n12x5               g098(.a(\a[18] ), .b(\b[17] ), .c(new_n193), .o1(new_n194));
  nor042aa1n12x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nand42aa1n04x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  aoai13aa1n06x5               g102(.a(new_n197), .b(new_n194), .c(new_n185), .d(new_n192), .o1(new_n198));
  aoi112aa1n02x5               g103(.a(new_n197), .b(new_n194), .c(new_n185), .d(new_n192), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g106(.a(new_n195), .o1(new_n202));
  nor042aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nand42aa1n06x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n198), .c(new_n202), .out0(\s[20] ));
  xorc02aa1n02x5               g111(.a(\a[17] ), .b(\b[16] ), .out0(new_n207));
  xorc02aa1n02x5               g112(.a(\a[18] ), .b(\b[17] ), .out0(new_n208));
  nano23aa1n09x5               g113(.a(new_n195), .b(new_n203), .c(new_n204), .d(new_n196), .out0(new_n209));
  nand23aa1n03x5               g114(.a(new_n209), .b(new_n207), .c(new_n208), .o1(new_n210));
  inv040aa1n02x5               g115(.a(new_n210), .o1(new_n211));
  aoi012aa1n02x5               g116(.a(new_n203), .b(new_n195), .c(new_n204), .o1(new_n212));
  aob012aa1n02x5               g117(.a(new_n212), .b(new_n209), .c(new_n194), .out0(new_n213));
  nor022aa1n08x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  nand02aa1n03x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n213), .c(new_n185), .d(new_n211), .o1(new_n217));
  aoi112aa1n02x5               g122(.a(new_n216), .b(new_n213), .c(new_n185), .d(new_n211), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n217), .b(new_n218), .out0(\s[21] ));
  inv000aa1d42x5               g124(.a(new_n214), .o1(new_n220));
  norp02aa1n02x5               g125(.a(\b[21] ), .b(\a[22] ), .o1(new_n221));
  nand42aa1n02x5               g126(.a(\b[21] ), .b(\a[22] ), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  xnbna2aa1n03x5               g128(.a(new_n223), .b(new_n217), .c(new_n220), .out0(\s[22] ));
  nona23aa1n09x5               g129(.a(new_n222), .b(new_n215), .c(new_n214), .d(new_n221), .out0(new_n225));
  nano32aa1n02x4               g130(.a(new_n225), .b(new_n209), .c(new_n208), .d(new_n207), .out0(new_n226));
  inv040aa1n02x5               g131(.a(new_n212), .o1(new_n227));
  inv040aa1n03x5               g132(.a(new_n225), .o1(new_n228));
  aoai13aa1n06x5               g133(.a(new_n228), .b(new_n227), .c(new_n209), .d(new_n194), .o1(new_n229));
  aoi012aa1n02x5               g134(.a(new_n221), .b(new_n214), .c(new_n222), .o1(new_n230));
  nanp02aa1n02x5               g135(.a(new_n229), .b(new_n230), .o1(new_n231));
  nor002aa1d32x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  tech160nm_finand02aa1n03p5x5 g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n233), .b(new_n232), .out0(new_n234));
  aoai13aa1n06x5               g139(.a(new_n234), .b(new_n231), .c(new_n185), .d(new_n226), .o1(new_n235));
  aoi112aa1n02x5               g140(.a(new_n234), .b(new_n231), .c(new_n185), .d(new_n226), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n235), .b(new_n236), .out0(\s[23] ));
  inv000aa1d42x5               g142(.a(new_n232), .o1(new_n238));
  nor022aa1n06x5               g143(.a(\b[23] ), .b(\a[24] ), .o1(new_n239));
  tech160nm_finand02aa1n03p5x5 g144(.a(\b[23] ), .b(\a[24] ), .o1(new_n240));
  norb02aa1n02x5               g145(.a(new_n240), .b(new_n239), .out0(new_n241));
  xnbna2aa1n03x5               g146(.a(new_n241), .b(new_n235), .c(new_n238), .out0(\s[24] ));
  nona23aa1d24x5               g147(.a(new_n240), .b(new_n233), .c(new_n232), .d(new_n239), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  nano22aa1n02x4               g149(.a(new_n210), .b(new_n228), .c(new_n244), .out0(new_n245));
  aoi012aa1n02x5               g150(.a(new_n239), .b(new_n232), .c(new_n240), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n243), .c(new_n229), .d(new_n230), .o1(new_n247));
  xorc02aa1n12x5               g152(.a(\a[25] ), .b(\b[24] ), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n247), .c(new_n185), .d(new_n245), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(new_n248), .b(new_n247), .c(new_n185), .d(new_n245), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n249), .b(new_n250), .out0(\s[25] ));
  orn002aa1n02x5               g156(.a(\a[25] ), .b(\b[24] ), .o(new_n252));
  xorc02aa1n12x5               g157(.a(\a[26] ), .b(\b[25] ), .out0(new_n253));
  xnbna2aa1n03x5               g158(.a(new_n253), .b(new_n249), .c(new_n252), .out0(\s[26] ));
  nand02aa1d04x5               g159(.a(new_n253), .b(new_n248), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  nona32aa1d24x5               g161(.a(new_n211), .b(new_n255), .c(new_n243), .d(new_n225), .out0(new_n257));
  oaoi13aa1n09x5               g162(.a(new_n257), .b(new_n184), .c(new_n131), .d(new_n178), .o1(new_n258));
  oaoi03aa1n12x5               g163(.a(\a[26] ), .b(\b[25] ), .c(new_n252), .o1(new_n259));
  aoi112aa1n06x5               g164(.a(new_n258), .b(new_n259), .c(new_n247), .d(new_n256), .o1(new_n260));
  nor022aa1n04x5               g165(.a(\b[26] ), .b(\a[27] ), .o1(new_n261));
  inv000aa1n06x5               g166(.a(new_n261), .o1(new_n262));
  nand42aa1n02x5               g167(.a(\b[26] ), .b(\a[27] ), .o1(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n260), .b(new_n263), .c(new_n262), .out0(\s[27] ));
  inv000aa1d42x5               g169(.a(new_n257), .o1(new_n265));
  nand02aa1d10x5               g170(.a(new_n185), .b(new_n265), .o1(new_n266));
  nand02aa1n04x5               g171(.a(new_n247), .b(new_n256), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n259), .o1(new_n268));
  nanb02aa1n02x5               g173(.a(new_n261), .b(new_n263), .out0(new_n269));
  aoi013aa1n02x4               g174(.a(new_n269), .b(new_n267), .c(new_n266), .d(new_n268), .o1(new_n270));
  tech160nm_fixnrc02aa1n05x5   g175(.a(\b[27] ), .b(\a[28] ), .out0(new_n271));
  nano22aa1n02x4               g176(.a(new_n270), .b(new_n262), .c(new_n271), .out0(new_n272));
  oaoi13aa1n03x5               g177(.a(new_n271), .b(new_n262), .c(new_n260), .d(new_n269), .o1(new_n273));
  norp02aa1n03x5               g178(.a(new_n273), .b(new_n272), .o1(\s[28] ));
  nano22aa1n03x7               g179(.a(new_n271), .b(new_n262), .c(new_n263), .out0(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[28] ), .b(\b[27] ), .c(new_n262), .carry(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[28] ), .b(\a[29] ), .out0(new_n278));
  oaoi13aa1n03x5               g183(.a(new_n278), .b(new_n277), .c(new_n260), .d(new_n276), .o1(new_n279));
  aoi013aa1n02x4               g184(.a(new_n276), .b(new_n267), .c(new_n266), .d(new_n268), .o1(new_n280));
  nano22aa1n03x5               g185(.a(new_n280), .b(new_n277), .c(new_n278), .out0(new_n281));
  norp02aa1n03x5               g186(.a(new_n279), .b(new_n281), .o1(\s[29] ));
  xorb03aa1n02x5               g187(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb02aa1n06x5               g188(.a(new_n278), .b(new_n275), .out0(new_n284));
  oao003aa1n02x5               g189(.a(\a[29] ), .b(\b[28] ), .c(new_n277), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[29] ), .b(\a[30] ), .out0(new_n286));
  oaoi13aa1n03x5               g191(.a(new_n286), .b(new_n285), .c(new_n260), .d(new_n284), .o1(new_n287));
  aoi013aa1n02x4               g192(.a(new_n284), .b(new_n267), .c(new_n266), .d(new_n268), .o1(new_n288));
  nano22aa1n03x5               g193(.a(new_n288), .b(new_n285), .c(new_n286), .out0(new_n289));
  norp02aa1n03x5               g194(.a(new_n287), .b(new_n289), .o1(\s[30] ));
  nona22aa1n02x4               g195(.a(new_n275), .b(new_n278), .c(new_n286), .out0(new_n291));
  aoi013aa1n02x4               g196(.a(new_n291), .b(new_n267), .c(new_n266), .d(new_n268), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[30] ), .b(\b[29] ), .c(new_n285), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[30] ), .b(\a[31] ), .out0(new_n294));
  nano22aa1n02x4               g199(.a(new_n292), .b(new_n293), .c(new_n294), .out0(new_n295));
  oaoi13aa1n03x5               g200(.a(new_n294), .b(new_n293), .c(new_n260), .d(new_n291), .o1(new_n296));
  norp02aa1n03x5               g201(.a(new_n296), .b(new_n295), .o1(\s[31] ));
  xnbna2aa1n03x5               g202(.a(new_n108), .b(new_n102), .c(new_n103), .out0(\s[3] ));
  orn002aa1n02x5               g203(.a(new_n108), .b(new_n104), .o(new_n299));
  xorc02aa1n02x5               g204(.a(\a[4] ), .b(\b[3] ), .out0(new_n300));
  norb02aa1n02x5               g205(.a(new_n102), .b(new_n300), .out0(new_n301));
  aoi022aa1n02x5               g206(.a(new_n299), .b(new_n301), .c(new_n110), .d(new_n300), .o1(\s[4] ));
  nanb02aa1n02x5               g207(.a(new_n127), .b(new_n111), .out0(new_n303));
  xnbna2aa1n03x5               g208(.a(new_n303), .b(new_n110), .c(new_n119), .out0(\s[5] ));
  aoi013aa1n02x4               g209(.a(new_n127), .b(new_n110), .c(new_n111), .d(new_n119), .o1(new_n305));
  xnrb03aa1n02x5               g210(.a(new_n305), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g211(.a(\a[6] ), .b(\b[5] ), .c(new_n305), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g213(.a(new_n123), .b(new_n124), .c(new_n307), .o1(new_n309));
  xnbna2aa1n03x5               g214(.a(new_n309), .b(new_n114), .c(new_n116), .out0(\s[8] ));
  xnrb03aa1n02x5               g215(.a(new_n131), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


