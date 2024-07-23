// Benchmark "adder" written by ABC on Wed Jul 17 22:13:01 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n315, new_n318, new_n320,
    new_n322, new_n323, new_n324;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nanp02aa1n04x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand22aa1n12x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nor042aa1n12x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  oaih12aa1n06x5               g004(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n100));
  tech160nm_fixnrc02aa1n04x5   g005(.a(\b[3] ), .b(\a[4] ), .out0(new_n101));
  norp02aa1n06x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand42aa1n03x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanb02aa1n06x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\a[3] ), .o1(new_n105));
  nanb02aa1n12x5               g010(.a(\b[2] ), .b(new_n105), .out0(new_n106));
  oao003aa1n06x5               g011(.a(\a[4] ), .b(\b[3] ), .c(new_n106), .carry(new_n107));
  oai013aa1d12x5               g012(.a(new_n107), .b(new_n101), .c(new_n100), .d(new_n104), .o1(new_n108));
  nor042aa1n06x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nanp02aa1n09x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand02aa1n04x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nona23aa1d18x5               g017(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .out0(new_n114));
  nor042aa1d18x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nanb02aa1n02x5               g021(.a(new_n115), .b(new_n116), .out0(new_n117));
  nor043aa1n06x5               g022(.a(new_n113), .b(new_n114), .c(new_n117), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\a[6] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[5] ), .o1(new_n120));
  oaoi03aa1n12x5               g025(.a(new_n119), .b(new_n120), .c(new_n115), .o1(new_n121));
  inv040aa1n03x5               g026(.a(new_n111), .o1(new_n122));
  oaoi03aa1n03x5               g027(.a(\a[8] ), .b(\b[7] ), .c(new_n122), .o1(new_n123));
  oabi12aa1n18x5               g028(.a(new_n123), .b(new_n113), .c(new_n121), .out0(new_n124));
  aoi012aa1n02x5               g029(.a(new_n124), .b(new_n108), .c(new_n118), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nanp02aa1n02x5               g032(.a(new_n108), .b(new_n118), .o1(new_n128));
  inv000aa1d42x5               g033(.a(new_n124), .o1(new_n129));
  nor042aa1n04x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nor042aa1n04x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand42aa1d28x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  tech160nm_fiaoi012aa1n04x5   g037(.a(new_n131), .b(new_n130), .c(new_n132), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[8] ), .b(\a[9] ), .o1(new_n134));
  nano23aa1d15x5               g039(.a(new_n130), .b(new_n131), .c(new_n132), .d(new_n134), .out0(new_n135));
  inv000aa1d42x5               g040(.a(new_n135), .o1(new_n136));
  aoai13aa1n02x5               g041(.a(new_n133), .b(new_n136), .c(new_n128), .d(new_n129), .o1(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  inv000aa1d42x5               g043(.a(\a[12] ), .o1(new_n139));
  nor022aa1n08x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nand02aa1n04x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  aoi012aa1n02x5               g046(.a(new_n140), .b(new_n137), .c(new_n141), .o1(new_n142));
  xorb03aa1n02x5               g047(.a(new_n142), .b(\b[11] ), .c(new_n139), .out0(\s[12] ));
  nor022aa1n04x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand02aa1d06x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nona23aa1n12x5               g050(.a(new_n145), .b(new_n141), .c(new_n140), .d(new_n144), .out0(new_n146));
  ao0012aa1n03x7               g051(.a(new_n144), .b(new_n140), .c(new_n145), .o(new_n147));
  oab012aa1n02x4               g052(.a(new_n147), .b(new_n146), .c(new_n133), .out0(new_n148));
  norb02aa1n02x5               g053(.a(new_n135), .b(new_n146), .out0(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n124), .c(new_n108), .d(new_n118), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n150), .b(new_n148), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g057(.a(\a[14] ), .o1(new_n153));
  nor042aa1n03x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand42aa1n08x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  aoi012aa1n02x5               g060(.a(new_n154), .b(new_n151), .c(new_n155), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(new_n153), .out0(\s[14] ));
  nor042aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nand02aa1n08x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nona23aa1n02x4               g064(.a(new_n159), .b(new_n155), .c(new_n154), .d(new_n158), .out0(new_n160));
  oabi12aa1n06x5               g065(.a(new_n147), .b(new_n146), .c(new_n133), .out0(new_n161));
  nano23aa1n06x5               g066(.a(new_n154), .b(new_n158), .c(new_n159), .d(new_n155), .out0(new_n162));
  aoi012aa1n02x5               g067(.a(new_n158), .b(new_n154), .c(new_n159), .o1(new_n163));
  inv000aa1n02x5               g068(.a(new_n163), .o1(new_n164));
  aoi012aa1n02x5               g069(.a(new_n164), .b(new_n161), .c(new_n162), .o1(new_n165));
  oai012aa1n04x7               g070(.a(new_n165), .b(new_n150), .c(new_n160), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand42aa1n16x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  nor042aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nand42aa1n08x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  inv000aa1d42x5               g078(.a(new_n173), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n168), .c(new_n166), .d(new_n170), .o1(new_n175));
  aoi112aa1n02x5               g080(.a(new_n168), .b(new_n174), .c(new_n166), .d(new_n169), .o1(new_n176));
  nanb02aa1n03x5               g081(.a(new_n176), .b(new_n175), .out0(\s[16] ));
  nano23aa1n03x7               g082(.a(new_n140), .b(new_n144), .c(new_n145), .d(new_n141), .out0(new_n178));
  nano23aa1n03x7               g083(.a(new_n168), .b(new_n171), .c(new_n172), .d(new_n169), .out0(new_n179));
  nand02aa1n04x5               g084(.a(new_n179), .b(new_n162), .o1(new_n180));
  nano22aa1n03x7               g085(.a(new_n180), .b(new_n135), .c(new_n178), .out0(new_n181));
  aoai13aa1n12x5               g086(.a(new_n181), .b(new_n124), .c(new_n108), .d(new_n118), .o1(new_n182));
  aoi012aa1n02x5               g087(.a(new_n171), .b(new_n168), .c(new_n172), .o1(new_n183));
  aob012aa1n06x5               g088(.a(new_n183), .b(new_n179), .c(new_n164), .out0(new_n184));
  aoib12aa1n12x5               g089(.a(new_n184), .b(new_n161), .c(new_n180), .out0(new_n185));
  nor042aa1d18x5               g090(.a(\b[16] ), .b(\a[17] ), .o1(new_n186));
  nand42aa1n03x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  norb02aa1n02x5               g092(.a(new_n187), .b(new_n186), .out0(new_n188));
  xnbna2aa1n03x5               g093(.a(new_n188), .b(new_n182), .c(new_n185), .out0(\s[17] ));
  nanp02aa1n06x5               g094(.a(new_n182), .b(new_n185), .o1(new_n190));
  tech160nm_fiaoi012aa1n05x5   g095(.a(new_n186), .b(new_n190), .c(new_n188), .o1(new_n191));
  inv040aa1d32x5               g096(.a(\a[18] ), .o1(new_n192));
  inv040aa1d28x5               g097(.a(\b[17] ), .o1(new_n193));
  nand02aa1d20x5               g098(.a(new_n193), .b(new_n192), .o1(new_n194));
  nand02aa1d28x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  xnbna2aa1n03x5               g100(.a(new_n191), .b(new_n195), .c(new_n194), .out0(\s[18] ));
  aob012aa1d18x5               g101(.a(new_n194), .b(new_n186), .c(new_n195), .out0(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  nano32aa1n06x5               g103(.a(new_n186), .b(new_n195), .c(new_n187), .d(new_n194), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  aoai13aa1n04x5               g105(.a(new_n198), .b(new_n200), .c(new_n182), .d(new_n185), .o1(new_n201));
  xorb03aa1n02x5               g106(.a(new_n201), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n06x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nand42aa1n06x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  norb02aa1n06x4               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  nor042aa1n06x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nand02aa1n08x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  norb02aa1n12x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  aoai13aa1n03x5               g115(.a(new_n210), .b(new_n204), .c(new_n201), .d(new_n206), .o1(new_n211));
  nanp02aa1n02x5               g116(.a(new_n201), .b(new_n206), .o1(new_n212));
  nona22aa1n03x5               g117(.a(new_n212), .b(new_n210), .c(new_n204), .out0(new_n213));
  nanp02aa1n03x5               g118(.a(new_n213), .b(new_n211), .o1(\s[20] ));
  nanp03aa1d12x5               g119(.a(new_n197), .b(new_n206), .c(new_n209), .o1(new_n215));
  aoi012aa1n09x5               g120(.a(new_n207), .b(new_n204), .c(new_n208), .o1(new_n216));
  nand02aa1n04x5               g121(.a(new_n215), .b(new_n216), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  nano23aa1n02x4               g123(.a(new_n204), .b(new_n207), .c(new_n208), .d(new_n205), .out0(new_n219));
  inv000aa1n03x5               g124(.a(new_n219), .o1(new_n220));
  nano32aa1n02x4               g125(.a(new_n220), .b(new_n188), .c(new_n194), .d(new_n195), .out0(new_n221));
  inv000aa1n02x5               g126(.a(new_n221), .o1(new_n222));
  aoai13aa1n04x5               g127(.a(new_n218), .b(new_n222), .c(new_n182), .d(new_n185), .o1(new_n223));
  xorb03aa1n02x5               g128(.a(new_n223), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  nand42aa1d28x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n226), .b(new_n225), .out0(new_n227));
  nor002aa1d32x5               g132(.a(\b[21] ), .b(\a[22] ), .o1(new_n228));
  nand02aa1d28x5               g133(.a(\b[21] ), .b(\a[22] ), .o1(new_n229));
  norb02aa1n02x5               g134(.a(new_n229), .b(new_n228), .out0(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  aoai13aa1n03x5               g136(.a(new_n231), .b(new_n225), .c(new_n223), .d(new_n227), .o1(new_n232));
  nand02aa1n02x5               g137(.a(new_n223), .b(new_n227), .o1(new_n233));
  nona22aa1n02x4               g138(.a(new_n233), .b(new_n231), .c(new_n225), .out0(new_n234));
  nanp02aa1n03x5               g139(.a(new_n234), .b(new_n232), .o1(\s[22] ));
  nano23aa1d15x5               g140(.a(new_n225), .b(new_n228), .c(new_n229), .d(new_n226), .out0(new_n236));
  nanp03aa1n02x5               g141(.a(new_n199), .b(new_n219), .c(new_n236), .o1(new_n237));
  ao0012aa1n03x7               g142(.a(new_n228), .b(new_n225), .c(new_n229), .o(new_n238));
  aoi012aa1n03x5               g143(.a(new_n238), .b(new_n217), .c(new_n236), .o1(new_n239));
  aoai13aa1n04x5               g144(.a(new_n239), .b(new_n237), .c(new_n182), .d(new_n185), .o1(new_n240));
  xorb03aa1n02x5               g145(.a(new_n240), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n09x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  nand42aa1d28x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n242), .out0(new_n244));
  nor042aa1n06x5               g149(.a(\b[23] ), .b(\a[24] ), .o1(new_n245));
  nand42aa1d28x5               g150(.a(\b[23] ), .b(\a[24] ), .o1(new_n246));
  nanb02aa1n02x5               g151(.a(new_n245), .b(new_n246), .out0(new_n247));
  aoai13aa1n03x5               g152(.a(new_n247), .b(new_n242), .c(new_n240), .d(new_n244), .o1(new_n248));
  nand42aa1n02x5               g153(.a(new_n240), .b(new_n244), .o1(new_n249));
  nona22aa1n02x5               g154(.a(new_n249), .b(new_n247), .c(new_n242), .out0(new_n250));
  nanp02aa1n03x5               g155(.a(new_n250), .b(new_n248), .o1(\s[24] ));
  nano23aa1d15x5               g156(.a(new_n242), .b(new_n245), .c(new_n246), .d(new_n243), .out0(new_n252));
  nand22aa1n09x5               g157(.a(new_n252), .b(new_n236), .o1(new_n253));
  tech160nm_fiao0012aa1n02p5x5 g158(.a(new_n245), .b(new_n242), .c(new_n246), .o(new_n254));
  tech160nm_fiaoi012aa1n03p5x5 g159(.a(new_n254), .b(new_n252), .c(new_n238), .o1(new_n255));
  aoai13aa1n12x5               g160(.a(new_n255), .b(new_n253), .c(new_n215), .d(new_n216), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n253), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n221), .b(new_n258), .o1(new_n259));
  aoai13aa1n04x5               g164(.a(new_n257), .b(new_n259), .c(new_n182), .d(new_n185), .o1(new_n260));
  xorb03aa1n02x5               g165(.a(new_n260), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .o1(new_n262));
  tech160nm_fixorc02aa1n02p5x5 g167(.a(\a[25] ), .b(\b[24] ), .out0(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[25] ), .b(\a[26] ), .out0(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n262), .c(new_n260), .d(new_n263), .o1(new_n265));
  nanp02aa1n02x5               g170(.a(new_n260), .b(new_n263), .o1(new_n266));
  nona22aa1n02x5               g171(.a(new_n266), .b(new_n264), .c(new_n262), .out0(new_n267));
  nanp02aa1n03x5               g172(.a(new_n267), .b(new_n265), .o1(\s[26] ));
  norb02aa1n06x4               g173(.a(new_n263), .b(new_n264), .out0(new_n269));
  nano32aa1n03x7               g174(.a(new_n253), .b(new_n199), .c(new_n269), .d(new_n219), .out0(new_n270));
  inv000aa1n02x5               g175(.a(new_n270), .o1(new_n271));
  inv000aa1d42x5               g176(.a(\a[26] ), .o1(new_n272));
  inv000aa1d42x5               g177(.a(\b[25] ), .o1(new_n273));
  oaoi03aa1n12x5               g178(.a(new_n272), .b(new_n273), .c(new_n262), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  aoi012aa1n06x5               g180(.a(new_n275), .b(new_n256), .c(new_n269), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n271), .c(new_n182), .d(new_n185), .o1(new_n277));
  xorb03aa1n03x5               g182(.a(new_n277), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nor042aa1n03x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  xorc02aa1n02x5               g184(.a(\a[27] ), .b(\b[26] ), .out0(new_n280));
  xnrc02aa1n02x5               g185(.a(\b[27] ), .b(\a[28] ), .out0(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n279), .c(new_n277), .d(new_n280), .o1(new_n282));
  nanp02aa1n06x5               g187(.a(new_n256), .b(new_n269), .o1(new_n283));
  nanp02aa1n06x5               g188(.a(new_n283), .b(new_n274), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n280), .b(new_n284), .c(new_n190), .d(new_n270), .o1(new_n285));
  nona22aa1n03x5               g190(.a(new_n285), .b(new_n281), .c(new_n279), .out0(new_n286));
  nanp02aa1n03x5               g191(.a(new_n282), .b(new_n286), .o1(\s[28] ));
  inv000aa1d42x5               g192(.a(\a[28] ), .o1(new_n288));
  inv000aa1d42x5               g193(.a(\b[27] ), .o1(new_n289));
  oaoi03aa1n09x5               g194(.a(new_n288), .b(new_n289), .c(new_n279), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  norb02aa1n02x5               g196(.a(new_n280), .b(new_n281), .out0(new_n292));
  aoai13aa1n02x5               g197(.a(new_n292), .b(new_n284), .c(new_n190), .d(new_n270), .o1(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  nona22aa1n03x5               g199(.a(new_n293), .b(new_n294), .c(new_n291), .out0(new_n295));
  aoai13aa1n03x5               g200(.a(new_n294), .b(new_n291), .c(new_n277), .d(new_n292), .o1(new_n296));
  nanp02aa1n03x5               g201(.a(new_n296), .b(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  tech160nm_fioaoi03aa1n03p5x5 g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .o1(new_n299));
  norb03aa1n02x5               g204(.a(new_n280), .b(new_n294), .c(new_n281), .out0(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[29] ), .b(\a[30] ), .out0(new_n301));
  aoai13aa1n03x5               g206(.a(new_n301), .b(new_n299), .c(new_n277), .d(new_n300), .o1(new_n302));
  aoai13aa1n03x5               g207(.a(new_n300), .b(new_n284), .c(new_n190), .d(new_n270), .o1(new_n303));
  nona22aa1n03x5               g208(.a(new_n303), .b(new_n301), .c(new_n299), .out0(new_n304));
  nanp02aa1n03x5               g209(.a(new_n302), .b(new_n304), .o1(\s[30] ));
  norb02aa1n02x5               g210(.a(new_n300), .b(new_n301), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n284), .c(new_n190), .d(new_n270), .o1(new_n307));
  nanb02aa1n02x5               g212(.a(new_n301), .b(new_n299), .out0(new_n308));
  oai012aa1n02x5               g213(.a(new_n308), .b(\b[29] ), .c(\a[30] ), .o1(new_n309));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  nona22aa1n02x4               g215(.a(new_n307), .b(new_n309), .c(new_n310), .out0(new_n311));
  aoai13aa1n03x5               g216(.a(new_n310), .b(new_n309), .c(new_n277), .d(new_n306), .o1(new_n312));
  nanp02aa1n03x5               g217(.a(new_n312), .b(new_n311), .o1(\s[31] ));
  xnbna2aa1n03x5               g218(.a(new_n100), .b(new_n103), .c(new_n106), .out0(\s[3] ));
  oaoi03aa1n02x5               g219(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g221(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g222(.a(new_n116), .b(new_n108), .c(new_n115), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(new_n119), .out0(\s[6] ));
  oaoi03aa1n02x5               g224(.a(\a[6] ), .b(\b[5] ), .c(new_n318), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanb02aa1n02x5               g226(.a(new_n109), .b(new_n110), .out0(new_n322));
  aoai13aa1n02x5               g227(.a(new_n322), .b(new_n111), .c(new_n320), .d(new_n112), .o1(new_n323));
  aoi112aa1n02x5               g228(.a(new_n322), .b(new_n111), .c(new_n320), .d(new_n112), .o1(new_n324));
  nanb02aa1n02x5               g229(.a(new_n324), .b(new_n323), .out0(\s[8] ));
  xnrb03aa1n02x5               g230(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


