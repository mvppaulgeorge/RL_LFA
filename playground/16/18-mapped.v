// Benchmark "adder" written by ABC on Wed Jul 17 20:18:03 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n308, new_n311, new_n312,
    new_n314, new_n315, new_n317, new_n319;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n16x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  and002aa1n02x5               g003(.a(\b[3] ), .b(\a[4] ), .o(new_n99));
  inv040aa1d32x5               g004(.a(\a[3] ), .o1(new_n100));
  inv040aa1n08x5               g005(.a(\b[2] ), .o1(new_n101));
  nand42aa1n03x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nand42aa1n03x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand22aa1n03x5               g008(.a(new_n102), .b(new_n103), .o1(new_n104));
  nand42aa1n08x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nor042aa1d18x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n24x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  oai012aa1d24x5               g012(.a(new_n105), .b(new_n106), .c(new_n107), .o1(new_n108));
  oa0022aa1n02x5               g013(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n109));
  oaoi13aa1n12x5               g014(.a(new_n99), .b(new_n109), .c(new_n108), .d(new_n104), .o1(new_n110));
  nor022aa1n16x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  tech160nm_finand02aa1n03p5x5 g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand42aa1n03x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  norp02aa1n12x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n113), .b(new_n112), .c(new_n114), .d(new_n111), .out0(new_n115));
  tech160nm_fixnrc02aa1n04x5   g020(.a(\b[5] ), .b(\a[6] ), .out0(new_n116));
  tech160nm_fixnrc02aa1n02p5x5 g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  nor043aa1n04x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\a[6] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[5] ), .o1(new_n120));
  nor042aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n122));
  tech160nm_fiaoi012aa1n03p5x5 g027(.a(new_n111), .b(new_n114), .c(new_n112), .o1(new_n123));
  tech160nm_fioai012aa1n05x5   g028(.a(new_n123), .b(new_n115), .c(new_n122), .o1(new_n124));
  nand02aa1d06x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n97), .out0(new_n126));
  aoai13aa1n04x5               g031(.a(new_n126), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n127));
  nor042aa1n04x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nanp02aa1n09x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g036(.a(new_n130), .o1(new_n132));
  aoi012aa1n02x5               g037(.a(new_n128), .b(new_n97), .c(new_n129), .o1(new_n133));
  aoai13aa1n06x5               g038(.a(new_n133), .b(new_n132), .c(new_n127), .d(new_n98), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n06x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand02aa1d24x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nor042aa1n06x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand02aa1n10x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n06x4               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  aoi112aa1n02x5               g045(.a(new_n140), .b(new_n136), .c(new_n134), .d(new_n137), .o1(new_n141));
  aoai13aa1n02x5               g046(.a(new_n140), .b(new_n136), .c(new_n134), .d(new_n137), .o1(new_n142));
  norb02aa1n03x4               g047(.a(new_n142), .b(new_n141), .out0(\s[12] ));
  nano23aa1n06x5               g048(.a(new_n136), .b(new_n138), .c(new_n139), .d(new_n137), .out0(new_n144));
  nano23aa1n06x5               g049(.a(new_n97), .b(new_n128), .c(new_n129), .d(new_n125), .out0(new_n145));
  nand22aa1n09x5               g050(.a(new_n145), .b(new_n144), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  aoai13aa1n06x5               g052(.a(new_n147), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n148));
  aoi112aa1n02x5               g053(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n149));
  norb02aa1n03x4               g054(.a(new_n137), .b(new_n136), .out0(new_n150));
  aoi112aa1n09x5               g055(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n151));
  oai112aa1n04x5               g056(.a(new_n140), .b(new_n150), .c(new_n151), .d(new_n128), .o1(new_n152));
  nona22aa1n12x5               g057(.a(new_n152), .b(new_n149), .c(new_n138), .out0(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  nor042aa1n06x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand42aa1n06x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  norb02aa1n03x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n157), .b(new_n148), .c(new_n154), .out0(\s[13] ));
  nanp02aa1n02x5               g063(.a(new_n148), .b(new_n154), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n155), .b(new_n159), .c(new_n156), .o1(new_n160));
  nor042aa1n04x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand22aa1n12x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  norb02aa1n03x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  xnrc02aa1n02x5               g068(.a(new_n160), .b(new_n163), .out0(\s[14] ));
  nona23aa1n08x5               g069(.a(new_n162), .b(new_n156), .c(new_n155), .d(new_n161), .out0(new_n165));
  ao0012aa1n12x5               g070(.a(new_n161), .b(new_n155), .c(new_n162), .o(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  aoai13aa1n04x5               g072(.a(new_n167), .b(new_n165), .c(new_n148), .d(new_n154), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nand42aa1n04x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nor042aa1n02x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nand42aa1n03x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  aoi112aa1n03x5               g080(.a(new_n175), .b(new_n170), .c(new_n168), .d(new_n171), .o1(new_n176));
  aoai13aa1n04x5               g081(.a(new_n175), .b(new_n170), .c(new_n168), .d(new_n171), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(\s[16] ));
  nano23aa1n06x5               g083(.a(new_n170), .b(new_n172), .c(new_n173), .d(new_n171), .out0(new_n179));
  nano32aa1n03x7               g084(.a(new_n146), .b(new_n179), .c(new_n157), .d(new_n163), .out0(new_n180));
  aoai13aa1n12x5               g085(.a(new_n180), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n181));
  norb02aa1n09x5               g086(.a(new_n179), .b(new_n165), .out0(new_n182));
  aoi112aa1n02x5               g087(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n183));
  nand42aa1n02x5               g088(.a(new_n179), .b(new_n166), .o1(new_n184));
  nona22aa1n03x5               g089(.a(new_n184), .b(new_n183), .c(new_n172), .out0(new_n185));
  aoi012aa1d24x5               g090(.a(new_n185), .b(new_n153), .c(new_n182), .o1(new_n186));
  xnrc02aa1n12x5               g091(.a(\b[16] ), .b(\a[17] ), .out0(new_n187));
  xobna2aa1n03x5               g092(.a(new_n187), .b(new_n181), .c(new_n186), .out0(\s[17] ));
  inv000aa1d42x5               g093(.a(\a[18] ), .o1(new_n189));
  nand02aa1d08x5               g094(.a(new_n181), .b(new_n186), .o1(new_n190));
  norp02aa1n02x5               g095(.a(\b[16] ), .b(\a[17] ), .o1(new_n191));
  aoib12aa1n06x5               g096(.a(new_n191), .b(new_n190), .c(new_n187), .out0(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xorc02aa1n02x5               g098(.a(\a[18] ), .b(\b[17] ), .out0(new_n194));
  norb02aa1n03x5               g099(.a(new_n194), .b(new_n187), .out0(new_n195));
  inv000aa1d42x5               g100(.a(new_n195), .o1(new_n196));
  inv000aa1d42x5               g101(.a(\b[17] ), .o1(new_n197));
  oao003aa1n02x5               g102(.a(new_n189), .b(new_n197), .c(new_n191), .carry(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  aoai13aa1n06x5               g104(.a(new_n199), .b(new_n196), .c(new_n181), .d(new_n186), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nand02aa1n08x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nor042aa1n06x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nand22aa1n06x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  norb02aa1n06x4               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  aoi112aa1n02x5               g112(.a(new_n207), .b(new_n203), .c(new_n200), .d(new_n204), .o1(new_n208));
  aoai13aa1n03x5               g113(.a(new_n207), .b(new_n203), .c(new_n200), .d(new_n204), .o1(new_n209));
  norb02aa1n03x4               g114(.a(new_n209), .b(new_n208), .out0(\s[20] ));
  nano23aa1n06x5               g115(.a(new_n203), .b(new_n205), .c(new_n206), .d(new_n204), .out0(new_n211));
  nanb03aa1n02x5               g116(.a(new_n187), .b(new_n211), .c(new_n194), .out0(new_n212));
  aoi112aa1n02x5               g117(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n213));
  norp02aa1n02x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  norb02aa1n03x4               g119(.a(new_n204), .b(new_n203), .out0(new_n215));
  aoi112aa1n03x5               g120(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n216));
  oai112aa1n04x5               g121(.a(new_n207), .b(new_n215), .c(new_n216), .d(new_n214), .o1(new_n217));
  nona22aa1n09x5               g122(.a(new_n217), .b(new_n213), .c(new_n205), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  aoai13aa1n06x5               g124(.a(new_n219), .b(new_n212), .c(new_n181), .d(new_n186), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  xnrc02aa1n12x5               g127(.a(\b[20] ), .b(\a[21] ), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[21] ), .b(\a[22] ), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoi112aa1n02x5               g131(.a(new_n222), .b(new_n226), .c(new_n220), .d(new_n224), .o1(new_n227));
  aoai13aa1n03x5               g132(.a(new_n226), .b(new_n222), .c(new_n220), .d(new_n224), .o1(new_n228));
  norb02aa1n03x4               g133(.a(new_n228), .b(new_n227), .out0(\s[22] ));
  nor002aa1n02x5               g134(.a(new_n225), .b(new_n223), .o1(new_n230));
  nand43aa1n03x5               g135(.a(new_n195), .b(new_n211), .c(new_n230), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n222), .o1(new_n232));
  oaoi03aa1n03x5               g137(.a(\a[22] ), .b(\b[21] ), .c(new_n232), .o1(new_n233));
  aoi012aa1n02x5               g138(.a(new_n233), .b(new_n218), .c(new_n230), .o1(new_n234));
  aoai13aa1n04x5               g139(.a(new_n234), .b(new_n231), .c(new_n181), .d(new_n186), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  nand22aa1n03x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  nor042aa1n04x5               g143(.a(\b[23] ), .b(\a[24] ), .o1(new_n239));
  nanp02aa1n02x5               g144(.a(\b[23] ), .b(\a[24] ), .o1(new_n240));
  norb02aa1n02x5               g145(.a(new_n240), .b(new_n239), .out0(new_n241));
  aoi112aa1n03x5               g146(.a(new_n237), .b(new_n241), .c(new_n235), .d(new_n238), .o1(new_n242));
  aoai13aa1n04x5               g147(.a(new_n241), .b(new_n237), .c(new_n235), .d(new_n238), .o1(new_n243));
  norb02aa1n03x4               g148(.a(new_n243), .b(new_n242), .out0(\s[24] ));
  nona23aa1n06x5               g149(.a(new_n240), .b(new_n238), .c(new_n237), .d(new_n239), .out0(new_n245));
  inv020aa1n02x5               g150(.a(new_n245), .o1(new_n246));
  nano22aa1n03x7               g151(.a(new_n212), .b(new_n230), .c(new_n246), .out0(new_n247));
  inv000aa1n02x5               g152(.a(new_n247), .o1(new_n248));
  nona32aa1n09x5               g153(.a(new_n218), .b(new_n245), .c(new_n225), .d(new_n223), .out0(new_n249));
  aoi112aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n250));
  aoi112aa1n02x7               g155(.a(new_n250), .b(new_n239), .c(new_n246), .d(new_n233), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(new_n249), .b(new_n251), .o1(new_n252));
  inv040aa1n03x5               g157(.a(new_n252), .o1(new_n253));
  aoai13aa1n02x5               g158(.a(new_n253), .b(new_n248), .c(new_n181), .d(new_n186), .o1(new_n254));
  xorb03aa1n02x5               g159(.a(new_n254), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n02x5               g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  xorc02aa1n02x5               g162(.a(\a[26] ), .b(\b[25] ), .out0(new_n258));
  aoi112aa1n03x4               g163(.a(new_n256), .b(new_n258), .c(new_n254), .d(new_n257), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n256), .o1(new_n260));
  aoai13aa1n04x5               g165(.a(new_n257), .b(new_n252), .c(new_n190), .d(new_n247), .o1(new_n261));
  aobi12aa1n02x5               g166(.a(new_n258), .b(new_n261), .c(new_n260), .out0(new_n262));
  norp02aa1n02x5               g167(.a(new_n262), .b(new_n259), .o1(\s[26] ));
  nanp02aa1n02x5               g168(.a(new_n258), .b(new_n257), .o1(new_n264));
  nano23aa1n06x5               g169(.a(new_n212), .b(new_n264), .c(new_n230), .d(new_n246), .out0(new_n265));
  oao003aa1n02x5               g170(.a(\a[26] ), .b(\b[25] ), .c(new_n260), .carry(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n264), .c(new_n249), .d(new_n251), .o1(new_n267));
  xorc02aa1n02x5               g172(.a(\a[27] ), .b(\b[26] ), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n267), .c(new_n190), .d(new_n265), .o1(new_n269));
  aoi112aa1n02x5               g174(.a(new_n267), .b(new_n268), .c(new_n190), .d(new_n265), .o1(new_n270));
  norb02aa1n02x5               g175(.a(new_n269), .b(new_n270), .out0(\s[27] ));
  nor042aa1n03x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  tech160nm_fixorc02aa1n04x5   g177(.a(\a[28] ), .b(\b[27] ), .out0(new_n273));
  nona22aa1n06x5               g178(.a(new_n269), .b(new_n273), .c(new_n272), .out0(new_n274));
  inv000aa1n03x5               g179(.a(new_n272), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n273), .o1(new_n276));
  tech160nm_fiaoi012aa1n03p5x5 g181(.a(new_n276), .b(new_n269), .c(new_n275), .o1(new_n277));
  norb02aa1n03x4               g182(.a(new_n274), .b(new_n277), .out0(\s[28] ));
  xorc02aa1n12x5               g183(.a(\a[29] ), .b(\b[28] ), .out0(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  and002aa1n02x5               g185(.a(new_n273), .b(new_n268), .o(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n267), .c(new_n190), .d(new_n265), .o1(new_n282));
  oao003aa1n09x5               g187(.a(\a[28] ), .b(\b[27] ), .c(new_n275), .carry(new_n283));
  aoi012aa1n03x5               g188(.a(new_n280), .b(new_n282), .c(new_n283), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n283), .o1(new_n285));
  nona22aa1n03x5               g190(.a(new_n282), .b(new_n285), .c(new_n279), .out0(new_n286));
  norb02aa1n03x4               g191(.a(new_n286), .b(new_n284), .out0(\s[29] ));
  xorb03aa1n02x5               g192(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g193(.a(new_n280), .b(new_n268), .c(new_n273), .out0(new_n289));
  aoai13aa1n03x5               g194(.a(new_n289), .b(new_n267), .c(new_n190), .d(new_n265), .o1(new_n290));
  oaoi03aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .c(new_n283), .o1(new_n291));
  inv000aa1n03x5               g196(.a(new_n291), .o1(new_n292));
  xorc02aa1n02x5               g197(.a(\a[30] ), .b(\b[29] ), .out0(new_n293));
  inv000aa1n02x5               g198(.a(new_n293), .o1(new_n294));
  tech160nm_fiaoi012aa1n02p5x5 g199(.a(new_n294), .b(new_n290), .c(new_n292), .o1(new_n295));
  nona22aa1n02x5               g200(.a(new_n290), .b(new_n291), .c(new_n293), .out0(new_n296));
  norb02aa1n03x4               g201(.a(new_n296), .b(new_n295), .out0(\s[30] ));
  nano32aa1n02x4               g202(.a(new_n294), .b(new_n279), .c(new_n273), .d(new_n268), .out0(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n267), .c(new_n190), .d(new_n265), .o1(new_n299));
  oaoi03aa1n02x5               g204(.a(\a[30] ), .b(\b[29] ), .c(new_n292), .o1(new_n300));
  xorc02aa1n02x5               g205(.a(\a[31] ), .b(\b[30] ), .out0(new_n301));
  nona22aa1n02x5               g206(.a(new_n299), .b(new_n300), .c(new_n301), .out0(new_n302));
  inv000aa1n02x5               g207(.a(new_n300), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n301), .o1(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n304), .b(new_n299), .c(new_n303), .o1(new_n305));
  norb02aa1n03x4               g210(.a(new_n302), .b(new_n305), .out0(\s[31] ));
  xnbna2aa1n03x5               g211(.a(new_n108), .b(new_n102), .c(new_n103), .out0(\s[3] ));
  oaoi03aa1n02x5               g212(.a(\a[3] ), .b(\b[2] ), .c(new_n108), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g214(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g215(.a(\a[5] ), .b(\b[4] ), .o(new_n311));
  nanb02aa1n02x5               g216(.a(new_n117), .b(new_n110), .out0(new_n312));
  xobna2aa1n03x5               g217(.a(new_n116), .b(new_n312), .c(new_n311), .out0(\s[6] ));
  aoi012aa1n02x5               g218(.a(new_n121), .b(new_n119), .c(new_n120), .o1(new_n314));
  aoi022aa1n02x5               g219(.a(new_n312), .b(new_n314), .c(\a[6] ), .d(\b[5] ), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g221(.a(new_n114), .b(new_n315), .c(new_n113), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  aoi112aa1n02x5               g223(.a(new_n126), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n319));
  norb02aa1n02x5               g224(.a(new_n127), .b(new_n319), .out0(\s[9] ));
endmodule

