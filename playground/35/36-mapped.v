// Benchmark "adder" written by ABC on Thu Jul 18 06:13:14 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n321, new_n322, new_n325, new_n326,
    new_n328, new_n330, new_n331, new_n332;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n24x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  aoi112aa1n03x5               g002(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n98));
  oab012aa1n04x5               g003(.a(new_n98), .b(\a[4] ), .c(\b[3] ), .out0(new_n99));
  inv040aa1d32x5               g004(.a(\a[2] ), .o1(new_n100));
  inv040aa1d32x5               g005(.a(\b[1] ), .o1(new_n101));
  nand02aa1d08x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  tech160nm_fioaoi03aa1n05x5   g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  xnrc02aa1n12x5               g008(.a(\b[2] ), .b(\a[3] ), .out0(new_n104));
  xnrc02aa1n12x5               g009(.a(\b[3] ), .b(\a[4] ), .out0(new_n105));
  oai013aa1n06x5               g010(.a(new_n99), .b(new_n103), .c(new_n105), .d(new_n104), .o1(new_n106));
  and002aa1n03x5               g011(.a(\b[6] ), .b(\a[7] ), .o(new_n107));
  tech160nm_finor002aa1n05x5   g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(new_n107), .b(new_n108), .o1(new_n109));
  tech160nm_fixnrc02aa1n05x5   g014(.a(\b[5] ), .b(\a[6] ), .out0(new_n110));
  xorc02aa1n12x5               g015(.a(\a[5] ), .b(\b[4] ), .out0(new_n111));
  xorc02aa1n12x5               g016(.a(\a[8] ), .b(\b[7] ), .out0(new_n112));
  inv040aa1n02x5               g017(.a(new_n112), .o1(new_n113));
  nano23aa1n03x7               g018(.a(new_n113), .b(new_n110), .c(new_n111), .d(new_n109), .out0(new_n114));
  aoi112aa1n03x5               g019(.a(new_n107), .b(new_n108), .c(\a[6] ), .d(\b[5] ), .o1(new_n115));
  oai022aa1n06x5               g020(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n116));
  inv000aa1d42x5               g021(.a(\a[7] ), .o1(new_n117));
  nanb02aa1n02x5               g022(.a(\b[6] ), .b(new_n117), .out0(new_n118));
  oaoi03aa1n09x5               g023(.a(\a[8] ), .b(\b[7] ), .c(new_n118), .o1(new_n119));
  aoi013aa1n06x4               g024(.a(new_n119), .b(new_n115), .c(new_n112), .d(new_n116), .o1(new_n120));
  inv030aa1n02x5               g025(.a(new_n120), .o1(new_n121));
  xorc02aa1n12x5               g026(.a(\a[9] ), .b(\b[8] ), .out0(new_n122));
  aoai13aa1n06x5               g027(.a(new_n122), .b(new_n121), .c(new_n106), .d(new_n114), .o1(new_n123));
  nor002aa1d32x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nanp02aa1n24x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nanb02aa1n02x5               g030(.a(new_n124), .b(new_n125), .out0(new_n126));
  xobna2aa1n03x5               g031(.a(new_n126), .b(new_n123), .c(new_n97), .out0(\s[10] ));
  nanb03aa1d24x5               g032(.a(new_n124), .b(new_n97), .c(new_n125), .out0(new_n128));
  inv000aa1d42x5               g033(.a(new_n128), .o1(new_n129));
  nand22aa1n03x5               g034(.a(new_n123), .b(new_n129), .o1(new_n130));
  nanp02aa1n04x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nor002aa1d32x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanb03aa1n02x5               g037(.a(new_n132), .b(new_n125), .c(new_n131), .out0(new_n133));
  nanb02aa1n03x5               g038(.a(new_n132), .b(new_n131), .out0(new_n134));
  oao003aa1n03x5               g039(.a(new_n100), .b(new_n101), .c(new_n102), .carry(new_n135));
  nona22aa1n09x5               g040(.a(new_n135), .b(new_n104), .c(new_n105), .out0(new_n136));
  xnrc02aa1n03x5               g041(.a(\b[6] ), .b(\a[7] ), .out0(new_n137));
  nona23aa1n09x5               g042(.a(new_n111), .b(new_n112), .c(new_n110), .d(new_n137), .out0(new_n138));
  aoai13aa1n12x5               g043(.a(new_n120), .b(new_n138), .c(new_n136), .d(new_n99), .o1(new_n139));
  aoai13aa1n02x5               g044(.a(new_n125), .b(new_n128), .c(new_n139), .d(new_n122), .o1(new_n140));
  aboi22aa1n03x5               g045(.a(new_n133), .b(new_n130), .c(new_n140), .d(new_n134), .out0(\s[11] ));
  nor042aa1d18x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand02aa1n06x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanb02aa1n12x5               g048(.a(new_n142), .b(new_n143), .out0(new_n144));
  inv040aa1n02x5               g049(.a(new_n144), .o1(new_n145));
  aoi113aa1n02x5               g050(.a(new_n145), .b(new_n132), .c(new_n130), .d(new_n131), .e(new_n125), .o1(new_n146));
  inv030aa1n02x5               g051(.a(new_n132), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n133), .c(new_n123), .d(new_n129), .o1(new_n148));
  tech160nm_fiaoi012aa1n02p5x5 g053(.a(new_n146), .b(new_n145), .c(new_n148), .o1(\s[12] ));
  nona23aa1d18x5               g054(.a(new_n145), .b(new_n122), .c(new_n126), .d(new_n134), .out0(new_n150));
  inv030aa1n02x5               g055(.a(new_n150), .o1(new_n151));
  aoai13aa1n06x5               g056(.a(new_n151), .b(new_n121), .c(new_n106), .d(new_n114), .o1(new_n152));
  nano32aa1n03x7               g057(.a(new_n144), .b(new_n147), .c(new_n131), .d(new_n125), .out0(new_n153));
  oaoi03aa1n06x5               g058(.a(\a[12] ), .b(\b[11] ), .c(new_n147), .o1(new_n154));
  tech160nm_fiao0012aa1n02p5x5 g059(.a(new_n154), .b(new_n153), .c(new_n128), .o(new_n155));
  nanb02aa1n06x5               g060(.a(new_n155), .b(new_n152), .out0(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nand02aa1d10x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  nor002aa1d32x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nand42aa1n04x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  aoi112aa1n02x5               g067(.a(new_n161), .b(new_n160), .c(new_n156), .d(new_n162), .o1(new_n163));
  inv000aa1n02x5               g068(.a(new_n161), .o1(new_n164));
  aob012aa1n02x5               g069(.a(new_n164), .b(new_n156), .c(new_n162), .out0(new_n165));
  aoi012aa1n02x5               g070(.a(new_n163), .b(new_n165), .c(new_n160), .o1(\s[14] ));
  nona23aa1d24x5               g071(.a(new_n159), .b(new_n162), .c(new_n161), .d(new_n158), .out0(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  aoai13aa1n06x5               g073(.a(new_n168), .b(new_n155), .c(new_n139), .d(new_n151), .o1(new_n169));
  oaoi03aa1n02x5               g074(.a(\a[14] ), .b(\b[13] ), .c(new_n164), .o1(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  xnrc02aa1n12x5               g076(.a(\b[14] ), .b(\a[15] ), .out0(new_n172));
  inv000aa1d42x5               g077(.a(new_n172), .o1(new_n173));
  xnbna2aa1n03x5               g078(.a(new_n173), .b(new_n169), .c(new_n171), .out0(\s[15] ));
  aoai13aa1n03x5               g079(.a(new_n173), .b(new_n170), .c(new_n156), .d(new_n168), .o1(new_n175));
  xnrc02aa1n12x5               g080(.a(\b[15] ), .b(\a[16] ), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  inv000aa1d42x5               g082(.a(\a[15] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\b[14] ), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(new_n179), .b(new_n178), .o1(new_n180));
  and002aa1n02x5               g085(.a(new_n176), .b(new_n180), .o(new_n181));
  aoai13aa1n03x5               g086(.a(new_n180), .b(new_n172), .c(new_n169), .d(new_n171), .o1(new_n182));
  aoi022aa1n03x5               g087(.a(new_n182), .b(new_n177), .c(new_n175), .d(new_n181), .o1(\s[16] ));
  nor043aa1d12x5               g088(.a(new_n167), .b(new_n172), .c(new_n176), .o1(new_n184));
  norb02aa1n12x5               g089(.a(new_n184), .b(new_n150), .out0(new_n185));
  aoai13aa1n06x5               g090(.a(new_n185), .b(new_n121), .c(new_n106), .d(new_n114), .o1(new_n186));
  norp02aa1n02x5               g091(.a(\b[15] ), .b(\a[16] ), .o1(new_n187));
  aoai13aa1n12x5               g092(.a(new_n184), .b(new_n154), .c(new_n153), .d(new_n128), .o1(new_n188));
  oai122aa1n02x7               g093(.a(new_n159), .b(new_n158), .c(new_n161), .d(new_n179), .e(new_n178), .o1(new_n189));
  aoi022aa1n02x5               g094(.a(new_n189), .b(new_n180), .c(\a[16] ), .d(\b[15] ), .o1(new_n190));
  nona22aa1d18x5               g095(.a(new_n188), .b(new_n190), .c(new_n187), .out0(new_n191));
  nanb02aa1n12x5               g096(.a(new_n191), .b(new_n186), .out0(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g098(.a(\a[17] ), .o1(new_n194));
  nanb02aa1n02x5               g099(.a(\b[16] ), .b(new_n194), .out0(new_n195));
  xorc02aa1n02x5               g100(.a(\a[17] ), .b(\b[16] ), .out0(new_n196));
  aoai13aa1n03x5               g101(.a(new_n196), .b(new_n191), .c(new_n139), .d(new_n185), .o1(new_n197));
  tech160nm_fixorc02aa1n02p5x5 g102(.a(\a[18] ), .b(\b[17] ), .out0(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n197), .c(new_n195), .out0(\s[18] ));
  inv000aa1d42x5               g104(.a(\a[18] ), .o1(new_n200));
  xroi22aa1d04x5               g105(.a(new_n194), .b(\b[16] ), .c(new_n200), .d(\b[17] ), .out0(new_n201));
  aoai13aa1n06x5               g106(.a(new_n201), .b(new_n191), .c(new_n139), .d(new_n185), .o1(new_n202));
  oaih22aa1d12x5               g107(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n203));
  oaib12aa1n18x5               g108(.a(new_n203), .b(new_n200), .c(\b[17] ), .out0(new_n204));
  nor002aa1d32x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nanp02aa1n09x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nanb02aa1n02x5               g111(.a(new_n205), .b(new_n206), .out0(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  xnbna2aa1n03x5               g113(.a(new_n208), .b(new_n202), .c(new_n204), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n03x5               g115(.a(\a[18] ), .b(\b[17] ), .c(new_n195), .o1(new_n211));
  aoai13aa1n03x5               g116(.a(new_n208), .b(new_n211), .c(new_n192), .d(new_n201), .o1(new_n212));
  nor002aa1d24x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nand02aa1d28x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  aoib12aa1n02x5               g120(.a(new_n205), .b(new_n214), .c(new_n213), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n205), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n217), .b(new_n207), .c(new_n202), .d(new_n204), .o1(new_n218));
  aoi022aa1n03x5               g123(.a(new_n218), .b(new_n215), .c(new_n212), .d(new_n216), .o1(\s[20] ));
  nona23aa1d18x5               g124(.a(new_n214), .b(new_n206), .c(new_n205), .d(new_n213), .out0(new_n220));
  nano22aa1n03x7               g125(.a(new_n220), .b(new_n196), .c(new_n198), .out0(new_n221));
  aoai13aa1n06x5               g126(.a(new_n221), .b(new_n191), .c(new_n139), .d(new_n185), .o1(new_n222));
  aoi012aa1n09x5               g127(.a(new_n213), .b(new_n205), .c(new_n214), .o1(new_n223));
  oai012aa1d24x5               g128(.a(new_n223), .b(new_n220), .c(new_n204), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  xnrc02aa1n12x5               g130(.a(\b[20] ), .b(\a[21] ), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  xnbna2aa1n03x5               g132(.a(new_n227), .b(new_n222), .c(new_n225), .out0(\s[21] ));
  aoai13aa1n06x5               g133(.a(new_n227), .b(new_n224), .c(new_n192), .d(new_n221), .o1(new_n229));
  xnrc02aa1n12x5               g134(.a(\b[21] ), .b(\a[22] ), .out0(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  nor042aa1n04x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  norb02aa1n02x5               g137(.a(new_n230), .b(new_n232), .out0(new_n233));
  inv040aa1n03x5               g138(.a(new_n232), .o1(new_n234));
  aoai13aa1n03x5               g139(.a(new_n234), .b(new_n226), .c(new_n222), .d(new_n225), .o1(new_n235));
  aoi022aa1n03x5               g140(.a(new_n235), .b(new_n231), .c(new_n229), .d(new_n233), .o1(\s[22] ));
  nano23aa1n03x7               g141(.a(new_n205), .b(new_n213), .c(new_n214), .d(new_n206), .out0(new_n237));
  nor042aa1d18x5               g142(.a(new_n230), .b(new_n226), .o1(new_n238));
  and003aa1n02x5               g143(.a(new_n201), .b(new_n238), .c(new_n237), .o(new_n239));
  aoai13aa1n06x5               g144(.a(new_n239), .b(new_n191), .c(new_n139), .d(new_n185), .o1(new_n240));
  oaoi03aa1n12x5               g145(.a(\a[22] ), .b(\b[21] ), .c(new_n234), .o1(new_n241));
  aoi012aa1d24x5               g146(.a(new_n241), .b(new_n224), .c(new_n238), .o1(new_n242));
  xnrc02aa1n12x5               g147(.a(\b[22] ), .b(\a[23] ), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  xnbna2aa1n03x5               g149(.a(new_n244), .b(new_n240), .c(new_n242), .out0(\s[23] ));
  inv000aa1d42x5               g150(.a(new_n242), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n244), .b(new_n246), .c(new_n192), .d(new_n239), .o1(new_n247));
  xorc02aa1n03x5               g152(.a(\a[24] ), .b(\b[23] ), .out0(new_n248));
  nor042aa1n06x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  norp02aa1n02x5               g154(.a(new_n248), .b(new_n249), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n249), .o1(new_n251));
  aoai13aa1n03x5               g156(.a(new_n251), .b(new_n243), .c(new_n240), .d(new_n242), .o1(new_n252));
  aoi022aa1n03x5               g157(.a(new_n252), .b(new_n248), .c(new_n247), .d(new_n250), .o1(\s[24] ));
  norb02aa1n09x5               g158(.a(new_n248), .b(new_n243), .out0(new_n254));
  inv000aa1n02x5               g159(.a(new_n254), .o1(new_n255));
  nano32aa1n03x7               g160(.a(new_n255), .b(new_n201), .c(new_n238), .d(new_n237), .out0(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n191), .c(new_n139), .d(new_n185), .o1(new_n257));
  inv000aa1n02x5               g162(.a(new_n223), .o1(new_n258));
  aoai13aa1n06x5               g163(.a(new_n238), .b(new_n258), .c(new_n237), .d(new_n211), .o1(new_n259));
  inv000aa1n02x5               g164(.a(new_n241), .o1(new_n260));
  oao003aa1n02x5               g165(.a(\a[24] ), .b(\b[23] ), .c(new_n251), .carry(new_n261));
  aoai13aa1n06x5               g166(.a(new_n261), .b(new_n255), .c(new_n259), .d(new_n260), .o1(new_n262));
  inv000aa1n02x5               g167(.a(new_n262), .o1(new_n263));
  xorc02aa1n12x5               g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  xnbna2aa1n03x5               g169(.a(new_n264), .b(new_n257), .c(new_n263), .out0(\s[25] ));
  aoai13aa1n03x5               g170(.a(new_n264), .b(new_n262), .c(new_n192), .d(new_n256), .o1(new_n266));
  xorc02aa1n02x5               g171(.a(\a[26] ), .b(\b[25] ), .out0(new_n267));
  nor042aa1n09x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  norp02aa1n02x5               g173(.a(new_n267), .b(new_n268), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n268), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n264), .o1(new_n271));
  aoai13aa1n03x5               g176(.a(new_n270), .b(new_n271), .c(new_n257), .d(new_n263), .o1(new_n272));
  aoi022aa1n03x5               g177(.a(new_n272), .b(new_n267), .c(new_n266), .d(new_n269), .o1(\s[26] ));
  and002aa1n12x5               g178(.a(new_n267), .b(new_n264), .o(new_n274));
  inv000aa1n02x5               g179(.a(new_n274), .o1(new_n275));
  nano32aa1n03x7               g180(.a(new_n275), .b(new_n221), .c(new_n238), .d(new_n254), .out0(new_n276));
  aoai13aa1n04x5               g181(.a(new_n276), .b(new_n191), .c(new_n139), .d(new_n185), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .c(new_n270), .carry(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  aoi012aa1n06x5               g184(.a(new_n279), .b(new_n262), .c(new_n274), .o1(new_n280));
  xorc02aa1n12x5               g185(.a(\a[27] ), .b(\b[26] ), .out0(new_n281));
  xnbna2aa1n03x5               g186(.a(new_n281), .b(new_n280), .c(new_n277), .out0(\s[27] ));
  aoai13aa1n02x5               g187(.a(new_n254), .b(new_n241), .c(new_n224), .d(new_n238), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n278), .b(new_n275), .c(new_n283), .d(new_n261), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n281), .b(new_n284), .c(new_n192), .d(new_n276), .o1(new_n285));
  xorc02aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .out0(new_n286));
  norp02aa1n02x5               g191(.a(\b[26] ), .b(\a[27] ), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n286), .b(new_n287), .o1(new_n288));
  inv000aa1n03x5               g193(.a(new_n287), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n281), .o1(new_n290));
  aoai13aa1n03x5               g195(.a(new_n289), .b(new_n290), .c(new_n280), .d(new_n277), .o1(new_n291));
  aoi022aa1n03x5               g196(.a(new_n291), .b(new_n286), .c(new_n285), .d(new_n288), .o1(\s[28] ));
  and002aa1n02x5               g197(.a(new_n286), .b(new_n281), .o(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n284), .c(new_n192), .d(new_n276), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n293), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n289), .carry(new_n296));
  aoai13aa1n02x5               g201(.a(new_n296), .b(new_n295), .c(new_n280), .d(new_n277), .o1(new_n297));
  xorc02aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .out0(new_n298));
  norb02aa1n02x5               g203(.a(new_n296), .b(new_n298), .out0(new_n299));
  aoi022aa1n03x5               g204(.a(new_n297), .b(new_n298), .c(new_n294), .d(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g206(.a(new_n290), .b(new_n286), .c(new_n298), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n284), .c(new_n192), .d(new_n276), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n302), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n305));
  aoai13aa1n03x5               g210(.a(new_n305), .b(new_n304), .c(new_n280), .d(new_n277), .o1(new_n306));
  xorc02aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .out0(new_n307));
  norb02aa1n02x5               g212(.a(new_n305), .b(new_n307), .out0(new_n308));
  aoi022aa1n03x5               g213(.a(new_n306), .b(new_n307), .c(new_n303), .d(new_n308), .o1(\s[30] ));
  nano32aa1n06x5               g214(.a(new_n290), .b(new_n307), .c(new_n286), .d(new_n298), .out0(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n284), .c(new_n192), .d(new_n276), .o1(new_n311));
  xorc02aa1n02x5               g216(.a(\a[31] ), .b(\b[30] ), .out0(new_n312));
  and002aa1n02x5               g217(.a(\b[29] ), .b(\a[30] ), .o(new_n313));
  oabi12aa1n02x5               g218(.a(new_n312), .b(\a[30] ), .c(\b[29] ), .out0(new_n314));
  oab012aa1n02x4               g219(.a(new_n314), .b(new_n305), .c(new_n313), .out0(new_n315));
  inv000aa1d42x5               g220(.a(new_n310), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n316), .c(new_n280), .d(new_n277), .o1(new_n318));
  aoi022aa1n03x5               g223(.a(new_n318), .b(new_n312), .c(new_n311), .d(new_n315), .o1(\s[31] ));
  xnrb03aa1n02x5               g224(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nanp02aa1n02x5               g225(.a(\b[2] ), .b(\a[3] ), .o1(new_n321));
  oai112aa1n02x5               g226(.a(new_n105), .b(new_n321), .c(new_n135), .d(new_n104), .o1(new_n322));
  oai012aa1n02x5               g227(.a(new_n322), .b(new_n106), .c(new_n105), .o1(\s[4] ));
  xorb03aa1n02x5               g228(.a(new_n106), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g229(.a(\b[4] ), .b(\a[5] ), .o1(new_n325));
  oaib12aa1n06x5               g230(.a(new_n325), .b(new_n106), .c(new_n111), .out0(new_n326));
  xnrb03aa1n02x5               g231(.a(new_n326), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n03x5               g232(.a(\a[6] ), .b(\b[5] ), .c(new_n326), .carry(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[6] ), .c(new_n117), .out0(\s[7] ));
  oaoi03aa1n02x5               g234(.a(\a[7] ), .b(\b[6] ), .c(new_n328), .o1(new_n330));
  norp02aa1n02x5               g235(.a(new_n112), .b(new_n107), .o1(new_n331));
  aob012aa1n02x5               g236(.a(new_n331), .b(new_n328), .c(new_n118), .out0(new_n332));
  oai012aa1n02x5               g237(.a(new_n332), .b(new_n330), .c(new_n113), .o1(\s[8] ));
  xorb03aa1n02x5               g238(.a(new_n139), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


