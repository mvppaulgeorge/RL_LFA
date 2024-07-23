// Benchmark "adder" written by ABC on Thu Jul 18 07:00:40 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n301, new_n304, new_n306,
    new_n307, new_n308, new_n309, new_n310, new_n312;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor022aa1n16x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nand02aa1n06x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nor002aa1d32x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nanp02aa1n04x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nona23aa1d18x5               g007(.a(new_n102), .b(new_n100), .c(new_n99), .d(new_n101), .out0(new_n103));
  nand42aa1d28x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  nor002aa1d32x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nor002aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  nand42aa1n08x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  nano23aa1n09x5               g012(.a(new_n106), .b(new_n105), .c(new_n107), .d(new_n104), .out0(new_n108));
  norb02aa1n06x5               g013(.a(new_n108), .b(new_n103), .out0(new_n109));
  nor022aa1n04x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  nanp02aa1n04x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nor022aa1n06x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  inv000aa1d42x5               g019(.a(\a[2] ), .o1(new_n115));
  inv000aa1d42x5               g020(.a(\b[1] ), .o1(new_n116));
  nand42aa1n08x5               g021(.a(\b[0] ), .b(\a[1] ), .o1(new_n117));
  tech160nm_fioaoi03aa1n05x5   g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  oai012aa1n02x7               g023(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n119));
  oaih12aa1n12x5               g024(.a(new_n119), .b(new_n114), .c(new_n118), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n109), .b(new_n120), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(new_n101), .b(new_n100), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\a[5] ), .o1(new_n123));
  inv000aa1d42x5               g028(.a(\b[4] ), .o1(new_n124));
  aoai13aa1n06x5               g029(.a(new_n104), .b(new_n105), .c(new_n124), .d(new_n123), .o1(new_n125));
  oai122aa1n12x5               g030(.a(new_n122), .b(new_n103), .c(new_n125), .d(\b[7] ), .e(\a[8] ), .o1(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  nand22aa1n03x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nanb02aa1n02x5               g033(.a(new_n97), .b(new_n128), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n98), .b(new_n129), .c(new_n127), .d(new_n121), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1d32x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n06x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  nor002aa1n12x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nand02aa1n08x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  oai012aa1d24x5               g041(.a(new_n136), .b(new_n135), .c(new_n97), .o1(new_n137));
  nano23aa1n06x5               g042(.a(new_n97), .b(new_n135), .c(new_n136), .d(new_n128), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n138), .b(new_n126), .c(new_n109), .d(new_n120), .o1(new_n139));
  xnbna2aa1n03x5               g044(.a(new_n134), .b(new_n139), .c(new_n137), .out0(\s[11] ));
  inv000aa1d42x5               g045(.a(new_n132), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(new_n121), .b(new_n127), .o1(new_n142));
  inv000aa1d42x5               g047(.a(new_n137), .o1(new_n143));
  aoai13aa1n02x5               g048(.a(new_n134), .b(new_n143), .c(new_n142), .d(new_n138), .o1(new_n144));
  nor002aa1d32x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nanp02aa1n04x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nanb02aa1n02x5               g051(.a(new_n145), .b(new_n146), .out0(new_n147));
  xobna2aa1n03x5               g052(.a(new_n147), .b(new_n144), .c(new_n141), .out0(\s[12] ));
  nona23aa1n09x5               g053(.a(new_n146), .b(new_n133), .c(new_n132), .d(new_n145), .out0(new_n149));
  norb02aa1n02x5               g054(.a(new_n138), .b(new_n149), .out0(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n126), .c(new_n109), .d(new_n120), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n145), .o1(new_n152));
  nand42aa1n02x5               g057(.a(new_n132), .b(new_n146), .o1(new_n153));
  oai112aa1n06x5               g058(.a(new_n153), .b(new_n152), .c(new_n149), .d(new_n137), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(new_n151), .b(new_n155), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n06x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n158), .b(new_n156), .c(new_n159), .o1(new_n160));
  xnrb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d32x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nona23aa1n06x5               g068(.a(new_n163), .b(new_n159), .c(new_n158), .d(new_n162), .out0(new_n164));
  oai012aa1n02x5               g069(.a(new_n163), .b(new_n162), .c(new_n158), .o1(new_n165));
  aoai13aa1n03x5               g070(.a(new_n165), .b(new_n164), .c(new_n151), .d(new_n155), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  xorc02aa1n02x5               g073(.a(\a[15] ), .b(\b[14] ), .out0(new_n169));
  xorc02aa1n02x5               g074(.a(\a[16] ), .b(\b[15] ), .out0(new_n170));
  aoi112aa1n02x5               g075(.a(new_n170), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n170), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(\s[16] ));
  nano23aa1n02x5               g078(.a(new_n132), .b(new_n145), .c(new_n146), .d(new_n133), .out0(new_n174));
  nanp02aa1n02x5               g079(.a(new_n170), .b(new_n169), .o1(new_n175));
  nano23aa1n03x7               g080(.a(new_n175), .b(new_n164), .c(new_n174), .d(new_n138), .out0(new_n176));
  aoai13aa1n12x5               g081(.a(new_n176), .b(new_n126), .c(new_n109), .d(new_n120), .o1(new_n177));
  nano22aa1n02x4               g082(.a(new_n164), .b(new_n169), .c(new_n170), .out0(new_n178));
  aoi112aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n179));
  oai022aa1n02x5               g084(.a(new_n175), .b(new_n165), .c(\b[15] ), .d(\a[16] ), .o1(new_n180));
  aoi112aa1n09x5               g085(.a(new_n180), .b(new_n179), .c(new_n154), .d(new_n178), .o1(new_n181));
  nand02aa1d10x5               g086(.a(new_n181), .b(new_n177), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g088(.a(\a[18] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\a[17] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\b[16] ), .o1(new_n186));
  oaoi03aa1n03x5               g091(.a(new_n185), .b(new_n186), .c(new_n182), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[17] ), .c(new_n184), .out0(\s[18] ));
  xroi22aa1d04x5               g093(.a(new_n185), .b(\b[16] ), .c(new_n184), .d(\b[17] ), .out0(new_n189));
  nanp02aa1n02x5               g094(.a(new_n186), .b(new_n185), .o1(new_n190));
  oaoi03aa1n02x5               g095(.a(\a[18] ), .b(\b[17] ), .c(new_n190), .o1(new_n191));
  nor002aa1n02x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  nand42aa1n06x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n191), .c(new_n182), .d(new_n189), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(new_n194), .b(new_n191), .c(new_n182), .d(new_n189), .o1(new_n196));
  norb02aa1n03x4               g101(.a(new_n195), .b(new_n196), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nand42aa1n03x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  nona22aa1n03x5               g106(.a(new_n195), .b(new_n201), .c(new_n192), .out0(new_n202));
  orn002aa1n02x5               g107(.a(\a[19] ), .b(\b[18] ), .o(new_n203));
  aobi12aa1n03x5               g108(.a(new_n201), .b(new_n195), .c(new_n203), .out0(new_n204));
  norb02aa1n03x4               g109(.a(new_n202), .b(new_n204), .out0(\s[20] ));
  nano23aa1n03x5               g110(.a(new_n192), .b(new_n199), .c(new_n200), .d(new_n193), .out0(new_n206));
  nanp02aa1n02x5               g111(.a(new_n189), .b(new_n206), .o1(new_n207));
  oai022aa1n02x5               g112(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n208));
  oaib12aa1n02x5               g113(.a(new_n208), .b(new_n184), .c(\b[17] ), .out0(new_n209));
  nona23aa1n09x5               g114(.a(new_n200), .b(new_n193), .c(new_n192), .d(new_n199), .out0(new_n210));
  oaoi03aa1n02x5               g115(.a(\a[20] ), .b(\b[19] ), .c(new_n203), .o1(new_n211));
  oabi12aa1n12x5               g116(.a(new_n211), .b(new_n210), .c(new_n209), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n207), .c(new_n181), .d(new_n177), .o1(new_n214));
  xorb03aa1n03x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  xorc02aa1n02x5               g122(.a(\a[22] ), .b(\b[21] ), .out0(new_n218));
  aoi112aa1n02x7               g123(.a(new_n216), .b(new_n218), .c(new_n214), .d(new_n217), .o1(new_n219));
  aoai13aa1n02x7               g124(.a(new_n218), .b(new_n216), .c(new_n214), .d(new_n217), .o1(new_n220));
  norb02aa1n03x4               g125(.a(new_n220), .b(new_n219), .out0(\s[22] ));
  inv000aa1d42x5               g126(.a(\a[21] ), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\a[22] ), .o1(new_n223));
  xroi22aa1d04x5               g128(.a(new_n222), .b(\b[20] ), .c(new_n223), .d(\b[21] ), .out0(new_n224));
  nanp03aa1n03x5               g129(.a(new_n224), .b(new_n189), .c(new_n206), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\b[21] ), .o1(new_n226));
  oaoi03aa1n12x5               g131(.a(new_n223), .b(new_n226), .c(new_n216), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n212), .c(new_n224), .o1(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n225), .c(new_n181), .d(new_n177), .o1(new_n230));
  xorb03aa1n03x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  xorc02aa1n03x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xorc02aa1n02x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  aoi112aa1n02x7               g139(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n235));
  aoai13aa1n02x7               g140(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n236));
  norb02aa1n03x4               g141(.a(new_n236), .b(new_n235), .out0(\s[24] ));
  and002aa1n02x7               g142(.a(new_n234), .b(new_n233), .o(new_n238));
  inv000aa1n02x5               g143(.a(new_n238), .o1(new_n239));
  nano32aa1n02x4               g144(.a(new_n239), .b(new_n224), .c(new_n189), .d(new_n206), .out0(new_n240));
  aoai13aa1n02x7               g145(.a(new_n224), .b(new_n211), .c(new_n206), .d(new_n191), .o1(new_n241));
  aoi112aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n242));
  oab012aa1n02x4               g147(.a(new_n242), .b(\a[24] ), .c(\b[23] ), .out0(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n239), .c(new_n241), .d(new_n227), .o1(new_n244));
  tech160nm_fixorc02aa1n05x5   g149(.a(\a[25] ), .b(\b[24] ), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n244), .c(new_n182), .d(new_n240), .o1(new_n246));
  aoi112aa1n02x5               g151(.a(new_n245), .b(new_n244), .c(new_n182), .d(new_n240), .o1(new_n247));
  norb02aa1n03x4               g152(.a(new_n246), .b(new_n247), .out0(\s[25] ));
  nor042aa1n03x5               g153(.a(\b[24] ), .b(\a[25] ), .o1(new_n249));
  xorc02aa1n02x5               g154(.a(\a[26] ), .b(\b[25] ), .out0(new_n250));
  nona22aa1n03x5               g155(.a(new_n246), .b(new_n250), .c(new_n249), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n249), .o1(new_n252));
  aobi12aa1n03x5               g157(.a(new_n250), .b(new_n246), .c(new_n252), .out0(new_n253));
  norb02aa1n02x7               g158(.a(new_n251), .b(new_n253), .out0(\s[26] ));
  and002aa1n06x5               g159(.a(new_n250), .b(new_n245), .o(new_n255));
  nano22aa1n03x7               g160(.a(new_n225), .b(new_n238), .c(new_n255), .out0(new_n256));
  nand02aa1d10x5               g161(.a(new_n182), .b(new_n256), .o1(new_n257));
  oao003aa1n02x5               g162(.a(\a[26] ), .b(\b[25] ), .c(new_n252), .carry(new_n258));
  aobi12aa1n06x5               g163(.a(new_n258), .b(new_n244), .c(new_n255), .out0(new_n259));
  xorc02aa1n02x5               g164(.a(\a[27] ), .b(\b[26] ), .out0(new_n260));
  xnbna2aa1n03x5               g165(.a(new_n260), .b(new_n257), .c(new_n259), .out0(\s[27] ));
  nor042aa1n03x5               g166(.a(\b[26] ), .b(\a[27] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  aobi12aa1n06x5               g168(.a(new_n260), .b(new_n257), .c(new_n259), .out0(new_n264));
  xnrc02aa1n02x5               g169(.a(\b[27] ), .b(\a[28] ), .out0(new_n265));
  nano22aa1n03x7               g170(.a(new_n264), .b(new_n263), .c(new_n265), .out0(new_n266));
  inv000aa1n02x5               g171(.a(new_n256), .o1(new_n267));
  aoi012aa1n06x5               g172(.a(new_n267), .b(new_n181), .c(new_n177), .o1(new_n268));
  aoai13aa1n02x7               g173(.a(new_n238), .b(new_n228), .c(new_n212), .d(new_n224), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n255), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n258), .b(new_n270), .c(new_n269), .d(new_n243), .o1(new_n271));
  tech160nm_fioai012aa1n03p5x5 g176(.a(new_n260), .b(new_n271), .c(new_n268), .o1(new_n272));
  aoi012aa1n03x5               g177(.a(new_n265), .b(new_n272), .c(new_n263), .o1(new_n273));
  norp02aa1n03x5               g178(.a(new_n273), .b(new_n266), .o1(\s[28] ));
  norb02aa1n02x5               g179(.a(new_n260), .b(new_n265), .out0(new_n275));
  aobi12aa1n06x5               g180(.a(new_n275), .b(new_n257), .c(new_n259), .out0(new_n276));
  oao003aa1n02x5               g181(.a(\a[28] ), .b(\b[27] ), .c(new_n263), .carry(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[28] ), .b(\a[29] ), .out0(new_n278));
  nano22aa1n03x7               g183(.a(new_n276), .b(new_n277), .c(new_n278), .out0(new_n279));
  oaih12aa1n02x5               g184(.a(new_n275), .b(new_n271), .c(new_n268), .o1(new_n280));
  aoi012aa1n03x5               g185(.a(new_n278), .b(new_n280), .c(new_n277), .o1(new_n281));
  nor042aa1n03x5               g186(.a(new_n281), .b(new_n279), .o1(\s[29] ));
  xorb03aa1n02x5               g187(.a(new_n117), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g188(.a(new_n260), .b(new_n278), .c(new_n265), .out0(new_n284));
  aobi12aa1n06x5               g189(.a(new_n284), .b(new_n257), .c(new_n259), .out0(new_n285));
  oao003aa1n02x5               g190(.a(\a[29] ), .b(\b[28] ), .c(new_n277), .carry(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[29] ), .b(\a[30] ), .out0(new_n287));
  nano22aa1n03x7               g192(.a(new_n285), .b(new_n286), .c(new_n287), .out0(new_n288));
  tech160nm_fioai012aa1n03p5x5 g193(.a(new_n284), .b(new_n271), .c(new_n268), .o1(new_n289));
  aoi012aa1n03x5               g194(.a(new_n287), .b(new_n289), .c(new_n286), .o1(new_n290));
  nor002aa1n02x5               g195(.a(new_n290), .b(new_n288), .o1(\s[30] ));
  norb02aa1n02x5               g196(.a(new_n284), .b(new_n287), .out0(new_n292));
  aobi12aa1n06x5               g197(.a(new_n292), .b(new_n257), .c(new_n259), .out0(new_n293));
  oao003aa1n02x5               g198(.a(\a[30] ), .b(\b[29] ), .c(new_n286), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[30] ), .b(\a[31] ), .out0(new_n295));
  nano22aa1n03x7               g200(.a(new_n293), .b(new_n294), .c(new_n295), .out0(new_n296));
  oaih12aa1n02x5               g201(.a(new_n292), .b(new_n271), .c(new_n268), .o1(new_n297));
  aoi012aa1n03x5               g202(.a(new_n295), .b(new_n297), .c(new_n294), .o1(new_n298));
  nor042aa1n03x5               g203(.a(new_n298), .b(new_n296), .o1(\s[31] ));
  xnrb03aa1n02x5               g204(.a(new_n118), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g205(.a(\a[3] ), .b(\b[2] ), .c(new_n118), .o1(new_n301));
  xorb03aa1n02x5               g206(.a(new_n301), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g207(.a(new_n120), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g208(.a(new_n123), .b(new_n124), .c(new_n120), .o1(new_n304));
  xnrb03aa1n02x5               g209(.a(new_n304), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g210(.a(new_n101), .b(new_n102), .out0(new_n306));
  inv000aa1d42x5               g211(.a(new_n108), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n120), .o1(new_n308));
  oaoi13aa1n02x5               g213(.a(new_n306), .b(new_n125), .c(new_n308), .d(new_n307), .o1(new_n309));
  oai112aa1n02x5               g214(.a(new_n306), .b(new_n125), .c(new_n308), .d(new_n307), .o1(new_n310));
  norb02aa1n02x5               g215(.a(new_n310), .b(new_n309), .out0(\s[7] ));
  norp02aa1n02x5               g216(.a(new_n309), .b(new_n101), .o1(new_n312));
  xnrb03aa1n02x5               g217(.a(new_n312), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g218(.a(new_n129), .b(new_n127), .c(new_n121), .out0(\s[9] ));
endmodule


