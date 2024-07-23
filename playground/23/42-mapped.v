// Benchmark "adder" written by ABC on Thu Jul 18 00:08:10 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n312, new_n313, new_n315, new_n316, new_n319,
    new_n321, new_n323, new_n324;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  and002aa1n06x5               g001(.a(\b[3] ), .b(\a[4] ), .o(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  oa0022aa1n02x5               g003(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  tech160nm_finand02aa1n05x5   g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nor002aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  norb03aa1n03x5               g007(.a(new_n101), .b(new_n100), .c(new_n102), .out0(new_n103));
  inv000aa1d42x5               g008(.a(\a[3] ), .o1(new_n104));
  inv000aa1d42x5               g009(.a(\b[2] ), .o1(new_n105));
  nanp02aa1n06x5               g010(.a(new_n105), .b(new_n104), .o1(new_n106));
  nand42aa1n03x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanp03aa1n02x5               g012(.a(new_n106), .b(new_n101), .c(new_n107), .o1(new_n108));
  tech160nm_fioai012aa1n03p5x5 g013(.a(new_n99), .b(new_n103), .c(new_n108), .o1(new_n109));
  nor042aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanb02aa1n02x5               g016(.a(new_n110), .b(new_n111), .out0(new_n112));
  nand02aa1d08x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nona23aa1n02x4               g021(.a(new_n113), .b(new_n116), .c(new_n115), .d(new_n114), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .out0(new_n118));
  nor043aa1n03x5               g023(.a(new_n117), .b(new_n118), .c(new_n112), .o1(new_n119));
  aoi112aa1n02x5               g024(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n120));
  norb02aa1n03x4               g025(.a(new_n111), .b(new_n110), .out0(new_n121));
  inv000aa1d42x5               g026(.a(new_n113), .o1(new_n122));
  xorc02aa1n02x5               g027(.a(\a[7] ), .b(\b[6] ), .out0(new_n123));
  norp02aa1n02x5               g028(.a(new_n115), .b(new_n114), .o1(new_n124));
  nona23aa1n06x5               g029(.a(new_n123), .b(new_n121), .c(new_n124), .d(new_n122), .out0(new_n125));
  nona22aa1n06x5               g030(.a(new_n125), .b(new_n120), .c(new_n110), .out0(new_n126));
  aoi013aa1n06x4               g031(.a(new_n126), .b(new_n119), .c(new_n109), .d(new_n98), .o1(new_n127));
  tech160nm_fioaoi03aa1n03p5x5 g032(.a(\a[9] ), .b(\b[8] ), .c(new_n127), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1d32x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand42aa1d28x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  norb02aa1n06x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  inv040aa1d32x5               g037(.a(\a[9] ), .o1(new_n133));
  inv040aa1d32x5               g038(.a(\b[8] ), .o1(new_n134));
  aoai13aa1n12x5               g039(.a(new_n131), .b(new_n130), .c(new_n133), .d(new_n134), .o1(new_n135));
  inv000aa1d42x5               g040(.a(new_n135), .o1(new_n136));
  nor002aa1n16x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nanp02aa1n12x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  aoai13aa1n06x5               g044(.a(new_n139), .b(new_n136), .c(new_n128), .d(new_n132), .o1(new_n140));
  aoi112aa1n02x5               g045(.a(new_n139), .b(new_n136), .c(new_n128), .d(new_n132), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(\s[11] ));
  inv000aa1d42x5               g047(.a(new_n137), .o1(new_n143));
  nor002aa1n04x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanp02aa1n12x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nanb02aa1n02x5               g050(.a(new_n144), .b(new_n145), .out0(new_n146));
  tech160nm_fiaoi012aa1n04x5   g051(.a(new_n146), .b(new_n140), .c(new_n143), .o1(new_n147));
  nanp03aa1n03x5               g052(.a(new_n140), .b(new_n143), .c(new_n146), .o1(new_n148));
  norb02aa1n03x4               g053(.a(new_n148), .b(new_n147), .out0(\s[12] ));
  oaoi13aa1n09x5               g054(.a(new_n97), .b(new_n99), .c(new_n103), .d(new_n108), .o1(new_n150));
  xnrc02aa1n02x5               g055(.a(\b[8] ), .b(\a[9] ), .out0(new_n151));
  nano23aa1n06x5               g056(.a(new_n137), .b(new_n144), .c(new_n145), .d(new_n138), .out0(new_n152));
  nanb03aa1d18x5               g057(.a(new_n151), .b(new_n152), .c(new_n132), .out0(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  aoai13aa1n02x5               g059(.a(new_n154), .b(new_n126), .c(new_n150), .d(new_n119), .o1(new_n155));
  nona23aa1n03x5               g060(.a(new_n145), .b(new_n138), .c(new_n137), .d(new_n144), .out0(new_n156));
  nanp02aa1n02x5               g061(.a(new_n137), .b(new_n145), .o1(new_n157));
  oai122aa1n12x5               g062(.a(new_n157), .b(new_n156), .c(new_n135), .d(\b[11] ), .e(\a[12] ), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  xorc02aa1n06x5               g064(.a(\a[13] ), .b(\b[12] ), .out0(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n160), .b(new_n155), .c(new_n159), .out0(\s[13] ));
  orn002aa1n02x5               g066(.a(\a[13] ), .b(\b[12] ), .o(new_n162));
  nanp03aa1n02x5               g067(.a(new_n119), .b(new_n109), .c(new_n98), .o1(new_n163));
  nanb02aa1n03x5               g068(.a(new_n126), .b(new_n163), .out0(new_n164));
  aoai13aa1n02x5               g069(.a(new_n160), .b(new_n158), .c(new_n164), .d(new_n154), .o1(new_n165));
  xorc02aa1n12x5               g070(.a(\a[14] ), .b(\b[13] ), .out0(new_n166));
  xnbna2aa1n03x5               g071(.a(new_n166), .b(new_n165), .c(new_n162), .out0(\s[14] ));
  and002aa1n02x5               g072(.a(new_n166), .b(new_n160), .o(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  oao003aa1n02x5               g074(.a(\a[14] ), .b(\b[13] ), .c(new_n162), .carry(new_n170));
  aoai13aa1n06x5               g075(.a(new_n170), .b(new_n169), .c(new_n155), .d(new_n159), .o1(new_n171));
  xorb03aa1n02x5               g076(.a(new_n171), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  xorc02aa1n02x5               g078(.a(\a[15] ), .b(\b[14] ), .out0(new_n174));
  xnrc02aa1n12x5               g079(.a(\b[15] ), .b(\a[16] ), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n173), .c(new_n171), .d(new_n174), .o1(new_n177));
  aoi112aa1n02x5               g082(.a(new_n173), .b(new_n176), .c(new_n171), .d(new_n174), .o1(new_n178));
  norb02aa1n03x4               g083(.a(new_n177), .b(new_n178), .out0(\s[16] ));
  xnrc02aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .out0(new_n180));
  norp02aa1n02x5               g085(.a(new_n175), .b(new_n180), .o1(new_n181));
  nano32aa1n06x5               g086(.a(new_n153), .b(new_n181), .c(new_n160), .d(new_n166), .out0(new_n182));
  aoai13aa1n06x5               g087(.a(new_n182), .b(new_n126), .c(new_n150), .d(new_n119), .o1(new_n183));
  nano32aa1n02x4               g088(.a(new_n175), .b(new_n174), .c(new_n166), .d(new_n160), .out0(new_n184));
  aoi112aa1n02x5               g089(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n185));
  nanb02aa1n02x5               g090(.a(new_n175), .b(new_n174), .out0(new_n186));
  oai022aa1n02x5               g091(.a(new_n186), .b(new_n170), .c(\b[15] ), .d(\a[16] ), .o1(new_n187));
  aoi112aa1n09x5               g092(.a(new_n187), .b(new_n185), .c(new_n158), .d(new_n184), .o1(new_n188));
  xorc02aa1n02x5               g093(.a(\a[17] ), .b(\b[16] ), .out0(new_n189));
  xnbna2aa1n03x5               g094(.a(new_n189), .b(new_n188), .c(new_n183), .out0(\s[17] ));
  inv040aa1d30x5               g095(.a(\a[17] ), .o1(new_n191));
  inv040aa1d30x5               g096(.a(\b[16] ), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(new_n192), .b(new_n191), .o1(new_n193));
  norp02aa1n02x5               g098(.a(\b[15] ), .b(\a[16] ), .o1(new_n194));
  nanp02aa1n03x5               g099(.a(new_n158), .b(new_n184), .o1(new_n195));
  norp03aa1n02x5               g100(.a(new_n170), .b(new_n180), .c(new_n175), .o1(new_n196));
  nona32aa1n03x5               g101(.a(new_n195), .b(new_n196), .c(new_n185), .d(new_n194), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n189), .b(new_n197), .c(new_n164), .d(new_n182), .o1(new_n198));
  nor002aa1n16x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nand42aa1d28x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nanb02aa1n06x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  xobna2aa1n03x5               g106(.a(new_n201), .b(new_n198), .c(new_n193), .out0(\s[18] ));
  nanp02aa1n02x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  nano22aa1d15x5               g108(.a(new_n201), .b(new_n193), .c(new_n203), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  aoai13aa1n12x5               g110(.a(new_n200), .b(new_n199), .c(new_n191), .d(new_n192), .o1(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n205), .c(new_n188), .d(new_n183), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n06x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nanp02aa1n03x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nor002aa1n04x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nand42aa1n02x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  aoai13aa1n03x5               g119(.a(new_n214), .b(new_n210), .c(new_n207), .d(new_n211), .o1(new_n215));
  aoi112aa1n02x7               g120(.a(new_n210), .b(new_n214), .c(new_n207), .d(new_n211), .o1(new_n216));
  norb02aa1n03x4               g121(.a(new_n215), .b(new_n216), .out0(\s[20] ));
  nano23aa1n06x5               g122(.a(new_n210), .b(new_n212), .c(new_n213), .d(new_n211), .out0(new_n218));
  nanp02aa1n02x5               g123(.a(new_n204), .b(new_n218), .o1(new_n219));
  nona23aa1n03x5               g124(.a(new_n213), .b(new_n211), .c(new_n210), .d(new_n212), .out0(new_n220));
  tech160nm_fiaoi012aa1n04x5   g125(.a(new_n212), .b(new_n210), .c(new_n213), .o1(new_n221));
  oai012aa1n12x5               g126(.a(new_n221), .b(new_n220), .c(new_n206), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoai13aa1n06x5               g128(.a(new_n223), .b(new_n219), .c(new_n188), .d(new_n183), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  xorc02aa1n02x5               g131(.a(\a[21] ), .b(\b[20] ), .out0(new_n227));
  xorc02aa1n02x5               g132(.a(\a[22] ), .b(\b[21] ), .out0(new_n228));
  aoai13aa1n03x5               g133(.a(new_n228), .b(new_n226), .c(new_n224), .d(new_n227), .o1(new_n229));
  aoi112aa1n02x7               g134(.a(new_n226), .b(new_n228), .c(new_n224), .d(new_n227), .o1(new_n230));
  norb02aa1n03x4               g135(.a(new_n229), .b(new_n230), .out0(\s[22] ));
  inv000aa1d42x5               g136(.a(\a[21] ), .o1(new_n232));
  inv000aa1d42x5               g137(.a(\a[22] ), .o1(new_n233));
  xroi22aa1d04x5               g138(.a(new_n232), .b(\b[20] ), .c(new_n233), .d(\b[21] ), .out0(new_n234));
  nanp03aa1n02x5               g139(.a(new_n234), .b(new_n204), .c(new_n218), .o1(new_n235));
  inv000aa1d42x5               g140(.a(\b[21] ), .o1(new_n236));
  tech160nm_fioaoi03aa1n03p5x5 g141(.a(new_n233), .b(new_n236), .c(new_n226), .o1(new_n237));
  inv000aa1n02x5               g142(.a(new_n237), .o1(new_n238));
  aoi012aa1n02x5               g143(.a(new_n238), .b(new_n222), .c(new_n234), .o1(new_n239));
  aoai13aa1n06x5               g144(.a(new_n239), .b(new_n235), .c(new_n188), .d(new_n183), .o1(new_n240));
  xorb03aa1n02x5               g145(.a(new_n240), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  tech160nm_fixorc02aa1n05x5   g147(.a(\a[23] ), .b(\b[22] ), .out0(new_n243));
  xorc02aa1n02x5               g148(.a(\a[24] ), .b(\b[23] ), .out0(new_n244));
  aoai13aa1n03x5               g149(.a(new_n244), .b(new_n242), .c(new_n240), .d(new_n243), .o1(new_n245));
  aoi112aa1n02x7               g150(.a(new_n242), .b(new_n244), .c(new_n240), .d(new_n243), .o1(new_n246));
  norb02aa1n03x4               g151(.a(new_n245), .b(new_n246), .out0(\s[24] ));
  and002aa1n02x5               g152(.a(new_n244), .b(new_n243), .o(new_n248));
  inv000aa1n02x5               g153(.a(new_n248), .o1(new_n249));
  nano32aa1n03x7               g154(.a(new_n249), .b(new_n234), .c(new_n218), .d(new_n204), .out0(new_n250));
  inv000aa1n02x5               g155(.a(new_n250), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n206), .o1(new_n252));
  inv040aa1n03x5               g157(.a(new_n221), .o1(new_n253));
  aoai13aa1n06x5               g158(.a(new_n234), .b(new_n253), .c(new_n218), .d(new_n252), .o1(new_n254));
  aoi112aa1n02x5               g159(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n255));
  oab012aa1n02x4               g160(.a(new_n255), .b(\a[24] ), .c(\b[23] ), .out0(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n249), .c(new_n254), .d(new_n237), .o1(new_n257));
  inv030aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  aoai13aa1n06x5               g163(.a(new_n258), .b(new_n251), .c(new_n188), .d(new_n183), .o1(new_n259));
  xorb03aa1n03x5               g164(.a(new_n259), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  xorc02aa1n12x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  xorc02aa1n12x5               g167(.a(\a[26] ), .b(\b[25] ), .out0(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n261), .c(new_n259), .d(new_n262), .o1(new_n264));
  aoi112aa1n02x7               g169(.a(new_n261), .b(new_n263), .c(new_n259), .d(new_n262), .o1(new_n265));
  norb02aa1n03x4               g170(.a(new_n264), .b(new_n265), .out0(\s[26] ));
  and002aa1n06x5               g171(.a(new_n263), .b(new_n262), .o(new_n267));
  nano22aa1n03x7               g172(.a(new_n235), .b(new_n248), .c(new_n267), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n197), .c(new_n164), .d(new_n182), .o1(new_n269));
  aoi112aa1n02x5               g174(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n270));
  oab012aa1n02x4               g175(.a(new_n270), .b(\a[26] ), .c(\b[25] ), .out0(new_n271));
  aobi12aa1n06x5               g176(.a(new_n271), .b(new_n257), .c(new_n267), .out0(new_n272));
  xorc02aa1n02x5               g177(.a(\a[27] ), .b(\b[26] ), .out0(new_n273));
  xnbna2aa1n03x5               g178(.a(new_n273), .b(new_n269), .c(new_n272), .out0(\s[27] ));
  norp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  inv040aa1n03x5               g180(.a(new_n275), .o1(new_n276));
  oaib12aa1n09x5               g181(.a(new_n188), .b(new_n127), .c(new_n182), .out0(new_n277));
  aoai13aa1n02x5               g182(.a(new_n248), .b(new_n238), .c(new_n222), .d(new_n234), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n267), .o1(new_n279));
  aoai13aa1n06x5               g184(.a(new_n271), .b(new_n279), .c(new_n278), .d(new_n256), .o1(new_n280));
  aoai13aa1n03x5               g185(.a(new_n273), .b(new_n280), .c(new_n277), .d(new_n268), .o1(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[27] ), .b(\a[28] ), .out0(new_n282));
  aoi012aa1n03x5               g187(.a(new_n282), .b(new_n281), .c(new_n276), .o1(new_n283));
  aobi12aa1n02x5               g188(.a(new_n273), .b(new_n269), .c(new_n272), .out0(new_n284));
  nano22aa1n02x4               g189(.a(new_n284), .b(new_n276), .c(new_n282), .out0(new_n285));
  nor002aa1n02x5               g190(.a(new_n283), .b(new_n285), .o1(\s[28] ));
  norb02aa1n02x5               g191(.a(new_n273), .b(new_n282), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n280), .c(new_n277), .d(new_n268), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n276), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  aoi012aa1n03x5               g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  aobi12aa1n02x5               g196(.a(new_n287), .b(new_n269), .c(new_n272), .out0(new_n292));
  nano22aa1n02x4               g197(.a(new_n292), .b(new_n289), .c(new_n290), .out0(new_n293));
  norp02aa1n03x5               g198(.a(new_n291), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g200(.a(new_n273), .b(new_n290), .c(new_n282), .out0(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n280), .c(new_n277), .d(new_n268), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  aoi012aa1n03x5               g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  aobi12aa1n02x5               g205(.a(new_n296), .b(new_n269), .c(new_n272), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n301), .b(new_n298), .c(new_n299), .out0(new_n302));
  nor002aa1n02x5               g207(.a(new_n300), .b(new_n302), .o1(\s[30] ));
  norb02aa1n02x5               g208(.a(new_n296), .b(new_n299), .out0(new_n304));
  aobi12aa1n02x5               g209(.a(new_n304), .b(new_n269), .c(new_n272), .out0(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  nano22aa1n02x4               g212(.a(new_n305), .b(new_n306), .c(new_n307), .out0(new_n308));
  aoai13aa1n03x5               g213(.a(new_n304), .b(new_n280), .c(new_n277), .d(new_n268), .o1(new_n309));
  aoi012aa1n03x5               g214(.a(new_n307), .b(new_n309), .c(new_n306), .o1(new_n310));
  nor002aa1n02x5               g215(.a(new_n310), .b(new_n308), .o1(\s[31] ));
  norp02aa1n02x5               g216(.a(new_n103), .b(new_n108), .o1(new_n312));
  aboi22aa1n03x5               g217(.a(new_n103), .b(new_n101), .c(new_n107), .d(new_n106), .out0(new_n313));
  norp02aa1n02x5               g218(.a(new_n313), .b(new_n312), .o1(\s[3] ));
  xnrc02aa1n02x5               g219(.a(\b[3] ), .b(\a[4] ), .out0(new_n315));
  nano22aa1n02x4               g220(.a(new_n312), .b(new_n106), .c(new_n315), .out0(new_n316));
  oaoi13aa1n02x5               g221(.a(new_n316), .b(new_n150), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xorb03aa1n02x5               g222(.a(new_n150), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi013aa1n02x4               g223(.a(new_n115), .b(new_n109), .c(new_n98), .d(new_n116), .o1(new_n319));
  xnrb03aa1n02x5               g224(.a(new_n319), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nona22aa1n02x4               g225(.a(new_n319), .b(new_n114), .c(new_n122), .out0(new_n321));
  xnbna2aa1n03x5               g226(.a(new_n118), .b(new_n321), .c(new_n113), .out0(\s[7] ));
  orn002aa1n02x5               g227(.a(\a[7] ), .b(\b[6] ), .o(new_n323));
  nona22aa1n02x4               g228(.a(new_n321), .b(new_n118), .c(new_n122), .out0(new_n324));
  xnbna2aa1n03x5               g229(.a(new_n121), .b(new_n324), .c(new_n323), .out0(\s[8] ));
  xorb03aa1n02x5               g230(.a(new_n127), .b(\b[8] ), .c(new_n133), .out0(\s[9] ));
endmodule


