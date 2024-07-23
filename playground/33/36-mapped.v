// Benchmark "adder" written by ABC on Thu Jul 18 05:11:45 2024

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
    new_n126, new_n127, new_n128, new_n130, new_n131, new_n132, new_n133,
    new_n135, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n152, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n273, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n304, new_n305, new_n306, new_n308, new_n309, new_n310, new_n312,
    new_n313, new_n315, new_n316, new_n317;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  tech160nm_fixorc02aa1n05x5   g002(.a(\a[3] ), .b(\b[2] ), .out0(new_n98));
  and002aa1n12x5               g003(.a(\b[0] ), .b(\a[1] ), .o(new_n99));
  oaoi03aa1n12x5               g004(.a(\a[2] ), .b(\b[1] ), .c(new_n99), .o1(new_n100));
  oaih22aa1n04x5               g005(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n101));
  aoi012aa1n12x5               g006(.a(new_n101), .b(new_n100), .c(new_n98), .o1(new_n102));
  nand42aa1n06x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor042aa1n06x5               g008(.a(\b[4] ), .b(\a[5] ), .o1(new_n104));
  nand42aa1n08x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nano22aa1n03x7               g010(.a(new_n104), .b(new_n103), .c(new_n105), .out0(new_n106));
  tech160nm_fixorc02aa1n04x5   g011(.a(\a[6] ), .b(\b[5] ), .out0(new_n107));
  oai022aa1n02x5               g012(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n108));
  nand22aa1n02x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nand42aa1n03x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nano22aa1n03x7               g015(.a(new_n108), .b(new_n109), .c(new_n110), .out0(new_n111));
  nanp03aa1n03x5               g016(.a(new_n111), .b(new_n106), .c(new_n107), .o1(new_n112));
  inv000aa1d42x5               g017(.a(\a[6] ), .o1(new_n113));
  aob012aa1n02x5               g018(.a(new_n104), .b(\b[5] ), .c(\a[6] ), .out0(new_n114));
  oaib12aa1n02x5               g019(.a(new_n114), .b(\b[5] ), .c(new_n113), .out0(new_n115));
  tech160nm_fixorc02aa1n04x5   g020(.a(\a[8] ), .b(\b[7] ), .out0(new_n116));
  nor042aa1n12x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  norb02aa1n06x4               g022(.a(new_n109), .b(new_n117), .out0(new_n118));
  inv000aa1d42x5               g023(.a(new_n117), .o1(new_n119));
  oaoi03aa1n02x5               g024(.a(\a[8] ), .b(\b[7] ), .c(new_n119), .o1(new_n120));
  aoi013aa1n06x4               g025(.a(new_n120), .b(new_n115), .c(new_n116), .d(new_n118), .o1(new_n121));
  oaih12aa1n12x5               g026(.a(new_n121), .b(new_n102), .c(new_n112), .o1(new_n122));
  oaib12aa1n06x5               g027(.a(new_n122), .b(new_n97), .c(\b[8] ), .out0(new_n123));
  oaib12aa1n02x5               g028(.a(new_n123), .b(\b[8] ), .c(new_n97), .out0(new_n124));
  xorb03aa1n02x5               g029(.a(new_n124), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  oaih22aa1d12x5               g030(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  aoi022aa1n03x5               g032(.a(new_n123), .b(new_n127), .c(\b[9] ), .d(\a[10] ), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  inv000aa1d42x5               g034(.a(\a[12] ), .o1(new_n130));
  nand42aa1n08x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nor042aa1n04x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  aoi012aa1n03x5               g037(.a(new_n132), .b(new_n128), .c(new_n131), .o1(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[11] ), .c(new_n130), .out0(\s[12] ));
  xorc02aa1n02x5               g039(.a(\a[12] ), .b(\b[11] ), .out0(new_n135));
  norp02aa1n02x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  nand42aa1n03x5               g041(.a(\b[9] ), .b(\a[10] ), .o1(new_n137));
  nano23aa1n02x4               g042(.a(new_n132), .b(new_n136), .c(new_n137), .d(new_n131), .out0(new_n138));
  xorc02aa1n02x5               g043(.a(\a[9] ), .b(\b[8] ), .out0(new_n139));
  nanp03aa1n02x5               g044(.a(new_n138), .b(new_n135), .c(new_n139), .o1(new_n140));
  nanb02aa1n06x5               g045(.a(new_n140), .b(new_n122), .out0(new_n141));
  nanp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  aoi022aa1n06x5               g047(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n143));
  oab012aa1n02x4               g048(.a(new_n132), .b(\a[12] ), .c(\b[11] ), .out0(new_n144));
  aobi12aa1n12x5               g049(.a(new_n144), .b(new_n143), .c(new_n126), .out0(new_n145));
  nanb02aa1n02x5               g050(.a(new_n145), .b(new_n142), .out0(new_n146));
  xnrc02aa1n12x5               g051(.a(\b[12] ), .b(\a[13] ), .out0(new_n147));
  xobna2aa1n03x5               g052(.a(new_n147), .b(new_n141), .c(new_n146), .out0(\s[13] ));
  inv000aa1d42x5               g053(.a(\a[13] ), .o1(new_n149));
  inv040aa1d28x5               g054(.a(\b[12] ), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n141), .b(new_n146), .o1(new_n151));
  oaoi03aa1n02x5               g056(.a(new_n149), .b(new_n150), .c(new_n151), .o1(new_n152));
  xnrb03aa1n02x5               g057(.a(new_n152), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d32x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nand02aa1d12x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nanb02aa1n12x5               g060(.a(new_n154), .b(new_n155), .out0(new_n156));
  nor042aa1n04x5               g061(.a(new_n147), .b(new_n156), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  aoai13aa1n04x5               g063(.a(new_n155), .b(new_n154), .c(new_n149), .d(new_n150), .o1(new_n159));
  aoai13aa1n04x5               g064(.a(new_n159), .b(new_n158), .c(new_n141), .d(new_n146), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n16x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  and002aa1n24x5               g067(.a(\b[14] ), .b(\a[15] ), .o(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  xnrc02aa1n02x5               g069(.a(\b[15] ), .b(\a[16] ), .out0(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n162), .c(new_n160), .d(new_n164), .o1(new_n166));
  aoi112aa1n02x5               g071(.a(new_n162), .b(new_n165), .c(new_n160), .d(new_n164), .o1(new_n167));
  nanb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(\s[16] ));
  xnrc02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .out0(new_n169));
  norp02aa1n02x5               g074(.a(new_n165), .b(new_n169), .o1(new_n170));
  nano22aa1n09x5               g075(.a(new_n140), .b(new_n170), .c(new_n157), .out0(new_n171));
  nanp02aa1n02x5               g076(.a(new_n122), .b(new_n171), .o1(new_n172));
  xorc02aa1n12x5               g077(.a(\a[16] ), .b(\b[15] ), .out0(new_n173));
  aoi112aa1n09x5               g078(.a(new_n163), .b(new_n162), .c(\a[13] ), .d(\b[12] ), .o1(new_n174));
  tech160nm_fioai012aa1n04x5   g079(.a(new_n142), .b(\b[12] ), .c(\a[13] ), .o1(new_n175));
  nona23aa1d18x5               g080(.a(new_n173), .b(new_n174), .c(new_n156), .d(new_n175), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n162), .o1(new_n177));
  inv000aa1d42x5               g082(.a(\a[16] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\b[15] ), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(new_n179), .b(new_n178), .o1(new_n180));
  ao0022aa1n03x5               g085(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o(new_n181));
  aoai13aa1n06x5               g086(.a(new_n180), .b(new_n181), .c(new_n159), .d(new_n177), .o1(new_n182));
  oabi12aa1n18x5               g087(.a(new_n182), .b(new_n176), .c(new_n145), .out0(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  tech160nm_fixorc02aa1n03p5x5 g089(.a(\a[17] ), .b(\b[16] ), .out0(new_n185));
  xnbna2aa1n03x5               g090(.a(new_n185), .b(new_n172), .c(new_n184), .out0(\s[17] ));
  orn002aa1n02x5               g091(.a(\a[17] ), .b(\b[16] ), .o(new_n187));
  aoai13aa1n02x5               g092(.a(new_n185), .b(new_n183), .c(new_n122), .d(new_n171), .o1(new_n188));
  xorc02aa1n02x5               g093(.a(\a[18] ), .b(\b[17] ), .out0(new_n189));
  xnbna2aa1n03x5               g094(.a(new_n189), .b(new_n188), .c(new_n187), .out0(\s[18] ));
  nor042aa1n02x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  and002aa1n02x5               g096(.a(new_n189), .b(new_n185), .o(new_n192));
  aoai13aa1n06x5               g097(.a(new_n192), .b(new_n183), .c(new_n122), .d(new_n171), .o1(new_n193));
  aoi112aa1n09x5               g098(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n194));
  nona22aa1n03x5               g099(.a(new_n193), .b(new_n194), .c(new_n191), .out0(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanp02aa1n09x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nor002aa1d32x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nand02aa1d16x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n15x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  aoai13aa1n02x5               g108(.a(new_n203), .b(new_n198), .c(new_n195), .d(new_n199), .o1(new_n204));
  aoi112aa1n03x4               g109(.a(new_n198), .b(new_n203), .c(new_n195), .d(new_n199), .o1(new_n205));
  nanb02aa1n03x5               g110(.a(new_n205), .b(new_n204), .out0(\s[20] ));
  nano23aa1n02x4               g111(.a(new_n198), .b(new_n200), .c(new_n201), .d(new_n199), .out0(new_n207));
  nand23aa1n03x5               g112(.a(new_n207), .b(new_n185), .c(new_n189), .o1(new_n208));
  norb02aa1n03x5               g113(.a(new_n199), .b(new_n198), .out0(new_n209));
  oai112aa1n06x5               g114(.a(new_n209), .b(new_n202), .c(new_n194), .d(new_n191), .o1(new_n210));
  oaih12aa1n02x5               g115(.a(new_n201), .b(new_n200), .c(new_n198), .o1(new_n211));
  nanp02aa1n02x5               g116(.a(new_n210), .b(new_n211), .o1(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n208), .c(new_n172), .d(new_n184), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xorc02aa1n12x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  nor042aa1d18x5               g122(.a(\b[21] ), .b(\a[22] ), .o1(new_n218));
  nand42aa1n06x5               g123(.a(\b[21] ), .b(\a[22] ), .o1(new_n219));
  nanb02aa1n06x5               g124(.a(new_n218), .b(new_n219), .out0(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n216), .c(new_n214), .d(new_n217), .o1(new_n221));
  aoi112aa1n03x5               g126(.a(new_n216), .b(new_n220), .c(new_n214), .d(new_n217), .o1(new_n222));
  nanb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(\s[22] ));
  nanb02aa1n12x5               g128(.a(new_n220), .b(new_n217), .out0(new_n224));
  nano32aa1n02x4               g129(.a(new_n224), .b(new_n207), .c(new_n189), .d(new_n185), .out0(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n183), .c(new_n122), .d(new_n171), .o1(new_n226));
  oai012aa1n02x7               g131(.a(new_n219), .b(new_n218), .c(new_n216), .o1(new_n227));
  aoai13aa1n12x5               g132(.a(new_n227), .b(new_n224), .c(new_n210), .d(new_n211), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[23] ), .b(\b[22] ), .out0(new_n230));
  xnbna2aa1n03x5               g135(.a(new_n230), .b(new_n226), .c(new_n229), .out0(\s[23] ));
  nanp02aa1n02x5               g136(.a(new_n226), .b(new_n229), .o1(new_n232));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n235), .b(new_n233), .c(new_n232), .d(new_n230), .o1(new_n236));
  aoi112aa1n02x5               g141(.a(new_n233), .b(new_n235), .c(new_n232), .d(new_n230), .o1(new_n237));
  nanb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(\s[24] ));
  inv000aa1d42x5               g143(.a(\a[23] ), .o1(new_n239));
  inv000aa1d42x5               g144(.a(\a[24] ), .o1(new_n240));
  xroi22aa1d04x5               g145(.a(new_n239), .b(\b[22] ), .c(new_n240), .d(\b[23] ), .out0(new_n241));
  norb03aa1n02x5               g146(.a(new_n241), .b(new_n208), .c(new_n224), .out0(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n183), .c(new_n122), .d(new_n171), .o1(new_n243));
  inv000aa1d42x5               g148(.a(\b[23] ), .o1(new_n244));
  oao003aa1n02x5               g149(.a(new_n240), .b(new_n244), .c(new_n233), .carry(new_n245));
  tech160nm_fiao0012aa1n02p5x5 g150(.a(new_n245), .b(new_n228), .c(new_n241), .o(new_n246));
  nanb02aa1n03x5               g151(.a(new_n246), .b(new_n243), .out0(new_n247));
  xorb03aa1n02x5               g152(.a(new_n247), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g153(.a(\b[24] ), .b(\a[25] ), .o1(new_n249));
  tech160nm_fixorc02aa1n03p5x5 g154(.a(\a[25] ), .b(\b[24] ), .out0(new_n250));
  xorc02aa1n12x5               g155(.a(\a[26] ), .b(\b[25] ), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  aoai13aa1n03x5               g157(.a(new_n252), .b(new_n249), .c(new_n247), .d(new_n250), .o1(new_n253));
  oaib12aa1n02x5               g158(.a(new_n250), .b(new_n246), .c(new_n243), .out0(new_n254));
  nona22aa1n02x5               g159(.a(new_n254), .b(new_n252), .c(new_n249), .out0(new_n255));
  nanp02aa1n03x5               g160(.a(new_n253), .b(new_n255), .o1(\s[26] ));
  norb02aa1n02x5               g161(.a(new_n217), .b(new_n220), .out0(new_n257));
  and002aa1n02x5               g162(.a(new_n251), .b(new_n250), .o(new_n258));
  nano32aa1n02x4               g163(.a(new_n208), .b(new_n258), .c(new_n257), .d(new_n241), .out0(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n183), .c(new_n122), .d(new_n171), .o1(new_n260));
  aoai13aa1n09x5               g165(.a(new_n258), .b(new_n245), .c(new_n228), .d(new_n241), .o1(new_n261));
  oai022aa1n02x5               g166(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n262));
  aob012aa1n02x5               g167(.a(new_n262), .b(\b[25] ), .c(\a[26] ), .out0(new_n263));
  nanp03aa1d12x5               g168(.a(new_n260), .b(new_n261), .c(new_n263), .o1(new_n264));
  xorb03aa1n03x5               g169(.a(new_n264), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nor042aa1d18x5               g170(.a(\b[26] ), .b(\a[27] ), .o1(new_n266));
  xorc02aa1n02x5               g171(.a(\a[27] ), .b(\b[26] ), .out0(new_n267));
  tech160nm_fixorc02aa1n03p5x5 g172(.a(\a[28] ), .b(\b[27] ), .out0(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  aoai13aa1n02x7               g174(.a(new_n269), .b(new_n266), .c(new_n264), .d(new_n267), .o1(new_n270));
  aoi112aa1n06x5               g175(.a(new_n266), .b(new_n269), .c(new_n264), .d(new_n267), .o1(new_n271));
  nanb02aa1n03x5               g176(.a(new_n271), .b(new_n270), .out0(\s[28] ));
  and002aa1n02x5               g177(.a(new_n268), .b(new_n267), .o(new_n273));
  nand42aa1n03x5               g178(.a(new_n264), .b(new_n273), .o1(new_n274));
  inv000aa1d42x5               g179(.a(\a[28] ), .o1(new_n275));
  inv020aa1d32x5               g180(.a(\b[27] ), .o1(new_n276));
  oaoi03aa1n12x5               g181(.a(new_n275), .b(new_n276), .c(new_n266), .o1(new_n277));
  xorc02aa1n12x5               g182(.a(\a[29] ), .b(\b[28] ), .out0(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  aoi012aa1n03x5               g184(.a(new_n279), .b(new_n274), .c(new_n277), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n277), .o1(new_n281));
  aoi112aa1n02x7               g186(.a(new_n278), .b(new_n281), .c(new_n264), .d(new_n273), .o1(new_n282));
  nor002aa1n02x5               g187(.a(new_n280), .b(new_n282), .o1(\s[29] ));
  xnrb03aa1n02x5               g188(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xorc02aa1n02x5               g189(.a(\a[30] ), .b(\b[29] ), .out0(new_n285));
  inv000aa1n02x5               g190(.a(new_n285), .o1(new_n286));
  nano22aa1n02x4               g191(.a(new_n279), .b(new_n267), .c(new_n268), .out0(new_n287));
  nand42aa1n03x5               g192(.a(new_n264), .b(new_n287), .o1(new_n288));
  oao003aa1n12x5               g193(.a(\a[29] ), .b(\b[28] ), .c(new_n277), .carry(new_n289));
  aoi012aa1n03x5               g194(.a(new_n286), .b(new_n288), .c(new_n289), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n289), .o1(new_n291));
  aoi112aa1n02x7               g196(.a(new_n285), .b(new_n291), .c(new_n264), .d(new_n287), .o1(new_n292));
  norp02aa1n03x5               g197(.a(new_n290), .b(new_n292), .o1(\s[30] ));
  xorc02aa1n02x5               g198(.a(\a[31] ), .b(\b[30] ), .out0(new_n294));
  inv000aa1d42x5               g199(.a(new_n294), .o1(new_n295));
  nano32aa1n02x4               g200(.a(new_n286), .b(new_n278), .c(new_n268), .d(new_n267), .out0(new_n296));
  nand42aa1n03x5               g201(.a(new_n264), .b(new_n296), .o1(new_n297));
  tech160nm_fioaoi03aa1n05x5   g202(.a(\a[30] ), .b(\b[29] ), .c(new_n289), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  tech160nm_fiaoi012aa1n05x5   g204(.a(new_n295), .b(new_n297), .c(new_n299), .o1(new_n300));
  aoi112aa1n03x4               g205(.a(new_n294), .b(new_n298), .c(new_n264), .d(new_n296), .o1(new_n301));
  norp02aa1n03x5               g206(.a(new_n300), .b(new_n301), .o1(\s[31] ));
  xorb03aa1n02x5               g207(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nanp02aa1n02x5               g208(.a(new_n100), .b(new_n98), .o1(new_n304));
  xorc02aa1n02x5               g209(.a(\a[4] ), .b(\b[3] ), .out0(new_n305));
  oab012aa1n02x4               g210(.a(new_n305), .b(\a[3] ), .c(\b[2] ), .out0(new_n306));
  aboi22aa1n03x5               g211(.a(new_n102), .b(new_n305), .c(new_n306), .d(new_n304), .out0(\s[4] ));
  aoai13aa1n02x5               g212(.a(new_n106), .b(new_n101), .c(new_n100), .d(new_n98), .o1(new_n308));
  aoai13aa1n02x5               g213(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n98), .o1(new_n309));
  oaib12aa1n02x5               g214(.a(new_n309), .b(new_n104), .c(new_n105), .out0(new_n310));
  and002aa1n02x5               g215(.a(new_n310), .b(new_n308), .o(\s[5] ));
  oaoi13aa1n02x5               g216(.a(new_n107), .b(new_n308), .c(\a[5] ), .d(\b[4] ), .o1(new_n312));
  oai112aa1n02x5               g217(.a(new_n308), .b(new_n107), .c(\b[4] ), .d(\a[5] ), .o1(new_n313));
  nanb02aa1n02x5               g218(.a(new_n312), .b(new_n313), .out0(\s[6] ));
  inv000aa1d42x5               g219(.a(\b[5] ), .o1(new_n315));
  oaoi13aa1n02x5               g220(.a(new_n118), .b(new_n313), .c(new_n113), .d(new_n315), .o1(new_n316));
  oai112aa1n02x5               g221(.a(new_n313), .b(new_n118), .c(new_n315), .d(new_n113), .o1(new_n317));
  norb02aa1n02x5               g222(.a(new_n317), .b(new_n316), .out0(\s[7] ));
  xnbna2aa1n03x5               g223(.a(new_n116), .b(new_n317), .c(new_n119), .out0(\s[8] ));
  xorb03aa1n02x5               g224(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


