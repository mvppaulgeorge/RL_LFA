// Benchmark "adder" written by ABC on Wed Jul 10 17:23:26 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n314, new_n315, new_n317, new_n318, new_n320, new_n321,
    new_n322, new_n323, new_n324;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  norp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aoi012aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1n02x4               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  aoi012aa1n02x5               g011(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n107));
  oai012aa1n02x5               g012(.a(new_n107), .b(new_n106), .c(new_n101), .o1(new_n108));
  xnrc02aa1n02x5               g013(.a(\b[5] ), .b(\a[6] ), .out0(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .out0(new_n115));
  norp03aa1n02x5               g020(.a(new_n114), .b(new_n115), .c(new_n109), .o1(new_n116));
  160nm_ficinv00aa1n08x5       g021(.clk(\b[5] ), .clkout(new_n117));
  oai022aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  oaib12aa1n02x5               g023(.a(new_n118), .b(new_n117), .c(\a[6] ), .out0(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(new_n112), .clkout(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[8] ), .b(\b[7] ), .c(new_n120), .o1(new_n121));
  oabi12aa1n02x5               g026(.a(new_n121), .b(new_n114), .c(new_n119), .out0(new_n122));
  xorc02aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n108), .d(new_n116), .o1(new_n124));
  xorc02aa1n02x5               g029(.a(\a[10] ), .b(\b[9] ), .out0(new_n125));
  xnbna2aa1n03x5               g030(.a(new_n125), .b(new_n124), .c(new_n97), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g031(.clk(\a[10] ), .clkout(new_n127));
  160nm_ficinv00aa1n08x5       g032(.clk(\b[9] ), .clkout(new_n128));
  norp02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  oai112aa1n02x5               g036(.a(new_n124), .b(new_n97), .c(\b[9] ), .d(\a[10] ), .o1(new_n132));
  oai112aa1n02x5               g037(.a(new_n132), .b(new_n131), .c(new_n128), .d(new_n127), .o1(new_n133));
  oaoi13aa1n02x5               g038(.a(new_n131), .b(new_n132), .c(new_n127), .d(new_n128), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n133), .b(new_n134), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g040(.clk(new_n129), .clkout(new_n136));
  norp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  xnbna2aa1n03x5               g044(.a(new_n139), .b(new_n133), .c(new_n136), .out0(\s[12] ));
  nanp02aa1n02x5               g045(.a(new_n108), .b(new_n116), .o1(new_n141));
  160nm_ficinv00aa1n08x5       g046(.clk(new_n122), .clkout(new_n142));
  nano23aa1n02x4               g047(.a(new_n129), .b(new_n137), .c(new_n138), .d(new_n130), .out0(new_n143));
  nanp03aa1n02x5               g048(.a(new_n143), .b(new_n123), .c(new_n125), .o1(new_n144));
  oaoi03aa1n02x5               g049(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n145));
  aoi012aa1n02x5               g050(.a(new_n137), .b(new_n129), .c(new_n138), .o1(new_n146));
  160nm_ficinv00aa1n08x5       g051(.clk(new_n146), .clkout(new_n147));
  aoi012aa1n02x5               g052(.a(new_n147), .b(new_n143), .c(new_n145), .o1(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n144), .c(new_n141), .d(new_n142), .o1(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n151), .b(new_n149), .c(new_n152), .o1(new_n153));
  xnrb03aa1n02x5               g058(.a(new_n153), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g059(.a(\b[14] ), .b(\a[15] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[14] ), .b(\a[15] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  norp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nano23aa1n02x4               g064(.a(new_n151), .b(new_n158), .c(new_n159), .d(new_n152), .out0(new_n160));
  oai012aa1n02x5               g065(.a(new_n159), .b(new_n158), .c(new_n151), .o1(new_n161));
  160nm_ficinv00aa1n08x5       g066(.clk(new_n161), .clkout(new_n162));
  aoai13aa1n02x5               g067(.a(new_n157), .b(new_n162), .c(new_n149), .d(new_n160), .o1(new_n163));
  aoi112aa1n02x5               g068(.a(new_n157), .b(new_n162), .c(new_n149), .d(new_n160), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(\s[15] ));
  norp02aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  nona22aa1n02x4               g073(.a(new_n163), .b(new_n168), .c(new_n155), .out0(new_n169));
  nanb02aa1n02x5               g074(.a(new_n166), .b(new_n167), .out0(new_n170));
  oaoi13aa1n02x5               g075(.a(new_n170), .b(new_n163), .c(\a[15] ), .d(\b[14] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n169), .b(new_n171), .out0(\s[16] ));
  nano23aa1n02x4               g077(.a(new_n155), .b(new_n166), .c(new_n167), .d(new_n156), .out0(new_n173));
  nanp02aa1n02x5               g078(.a(new_n173), .b(new_n160), .o1(new_n174));
  nano32aa1n02x4               g079(.a(new_n174), .b(new_n143), .c(new_n125), .d(new_n123), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n122), .c(new_n108), .d(new_n116), .o1(new_n176));
  160nm_ficinv00aa1n08x5       g081(.clk(new_n166), .clkout(new_n177));
  nanp02aa1n02x5               g082(.a(new_n173), .b(new_n162), .o1(new_n178));
  aoi112aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(new_n179), .clkout(new_n180));
  norp02aa1n02x5               g085(.a(new_n148), .b(new_n174), .o1(new_n181));
  nano32aa1n02x4               g086(.a(new_n181), .b(new_n180), .c(new_n178), .d(new_n177), .out0(new_n182));
  nanp02aa1n02x5               g087(.a(new_n182), .b(new_n176), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[18] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[17] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\b[16] ), .clkout(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  xroi22aa1d04x5               g094(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n190));
  nanp02aa1n02x5               g095(.a(new_n187), .b(new_n186), .o1(new_n191));
  oaoi03aa1n02x5               g096(.a(\a[18] ), .b(\b[17] ), .c(new_n191), .o1(new_n192));
  norp02aa1n02x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n192), .c(new_n183), .d(new_n190), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(new_n195), .b(new_n192), .c(new_n183), .d(new_n190), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  nona22aa1n02x4               g107(.a(new_n196), .b(new_n202), .c(new_n193), .out0(new_n203));
  160nm_ficinv00aa1n08x5       g108(.clk(new_n193), .clkout(new_n204));
  aobi12aa1n02x5               g109(.a(new_n202), .b(new_n196), .c(new_n204), .out0(new_n205));
  norb02aa1n02x5               g110(.a(new_n203), .b(new_n205), .out0(\s[20] ));
  nano23aa1n02x4               g111(.a(new_n193), .b(new_n200), .c(new_n201), .d(new_n194), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n190), .b(new_n207), .o1(new_n208));
  oai022aa1n02x5               g113(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n209));
  oaib12aa1n02x5               g114(.a(new_n209), .b(new_n185), .c(\b[17] ), .out0(new_n210));
  nona23aa1n02x4               g115(.a(new_n201), .b(new_n194), .c(new_n193), .d(new_n200), .out0(new_n211));
  aoi012aa1n02x5               g116(.a(new_n200), .b(new_n193), .c(new_n201), .o1(new_n212));
  oai012aa1n02x5               g117(.a(new_n212), .b(new_n211), .c(new_n210), .o1(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n208), .c(new_n182), .d(new_n176), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xorc02aa1n02x5               g123(.a(\a[22] ), .b(\b[21] ), .out0(new_n219));
  aoi112aa1n02x5               g124(.a(new_n217), .b(new_n219), .c(new_n215), .d(new_n218), .o1(new_n220));
  aoai13aa1n02x5               g125(.a(new_n219), .b(new_n217), .c(new_n215), .d(new_n218), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g127(.clk(\a[21] ), .clkout(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(\a[22] ), .clkout(new_n224));
  xroi22aa1d04x5               g129(.a(new_n223), .b(\b[20] ), .c(new_n224), .d(\b[21] ), .out0(new_n225));
  nanp03aa1n02x5               g130(.a(new_n225), .b(new_n190), .c(new_n207), .o1(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(\b[21] ), .clkout(new_n227));
  oao003aa1n02x5               g132(.a(new_n224), .b(new_n227), .c(new_n217), .carry(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n213), .c(new_n225), .o1(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n226), .c(new_n182), .d(new_n176), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xorc02aa1n02x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  aoi112aa1n02x5               g139(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(\s[24] ));
  160nm_ficinv00aa1n08x5       g142(.clk(\a[23] ), .clkout(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(\a[24] ), .clkout(new_n239));
  xroi22aa1d04x5               g144(.a(new_n238), .b(\b[22] ), .c(new_n239), .d(\b[23] ), .out0(new_n240));
  nano22aa1n02x4               g145(.a(new_n208), .b(new_n225), .c(new_n240), .out0(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n212), .clkout(new_n242));
  aoai13aa1n02x5               g147(.a(new_n225), .b(new_n242), .c(new_n207), .d(new_n192), .o1(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n228), .clkout(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n240), .clkout(new_n245));
  aoi112aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n246));
  aoib12aa1n02x5               g151(.a(new_n246), .b(new_n239), .c(\b[23] ), .out0(new_n247));
  aoai13aa1n02x5               g152(.a(new_n247), .b(new_n245), .c(new_n243), .d(new_n244), .o1(new_n248));
  xorc02aa1n02x5               g153(.a(\a[25] ), .b(\b[24] ), .out0(new_n249));
  aoai13aa1n02x5               g154(.a(new_n249), .b(new_n248), .c(new_n183), .d(new_n241), .o1(new_n250));
  aoi112aa1n02x5               g155(.a(new_n249), .b(new_n248), .c(new_n183), .d(new_n241), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n250), .b(new_n251), .out0(\s[25] ));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  nona22aa1n02x4               g159(.a(new_n250), .b(new_n254), .c(new_n253), .out0(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n253), .clkout(new_n256));
  aobi12aa1n02x5               g161(.a(new_n254), .b(new_n250), .c(new_n256), .out0(new_n257));
  norb02aa1n02x5               g162(.a(new_n255), .b(new_n257), .out0(\s[26] ));
  nanp02aa1n02x5               g163(.a(new_n141), .b(new_n142), .o1(new_n259));
  nona23aa1n02x4               g164(.a(new_n159), .b(new_n152), .c(new_n151), .d(new_n158), .out0(new_n260));
  nano22aa1n02x4               g165(.a(new_n260), .b(new_n157), .c(new_n168), .out0(new_n261));
  aoai13aa1n02x5               g166(.a(new_n261), .b(new_n147), .c(new_n143), .d(new_n145), .o1(new_n262));
  nona23aa1n02x4               g167(.a(new_n262), .b(new_n178), .c(new_n166), .d(new_n179), .out0(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(\a[25] ), .clkout(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(\a[26] ), .clkout(new_n265));
  xroi22aa1d04x5               g170(.a(new_n264), .b(\b[24] ), .c(new_n265), .d(\b[25] ), .out0(new_n266));
  nano32aa1n02x4               g171(.a(new_n208), .b(new_n266), .c(new_n225), .d(new_n240), .out0(new_n267));
  aoai13aa1n02x5               g172(.a(new_n267), .b(new_n263), .c(new_n259), .d(new_n175), .o1(new_n268));
  oao003aa1n02x5               g173(.a(\a[26] ), .b(\b[25] ), .c(new_n256), .carry(new_n269));
  aobi12aa1n02x5               g174(.a(new_n269), .b(new_n248), .c(new_n266), .out0(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnbna2aa1n03x5               g176(.a(new_n271), .b(new_n270), .c(new_n268), .out0(\s[27] ));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n273), .clkout(new_n274));
  aobi12aa1n02x5               g179(.a(new_n271), .b(new_n270), .c(new_n268), .out0(new_n275));
  xnrc02aa1n02x5               g180(.a(\b[27] ), .b(\a[28] ), .out0(new_n276));
  nano22aa1n02x4               g181(.a(new_n275), .b(new_n274), .c(new_n276), .out0(new_n277));
  aobi12aa1n02x5               g182(.a(new_n267), .b(new_n182), .c(new_n176), .out0(new_n278));
  aoai13aa1n02x5               g183(.a(new_n240), .b(new_n228), .c(new_n213), .d(new_n225), .o1(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n266), .clkout(new_n280));
  aoai13aa1n02x5               g185(.a(new_n269), .b(new_n280), .c(new_n279), .d(new_n247), .o1(new_n281));
  oai012aa1n02x5               g186(.a(new_n271), .b(new_n281), .c(new_n278), .o1(new_n282));
  aoi012aa1n02x5               g187(.a(new_n276), .b(new_n282), .c(new_n274), .o1(new_n283));
  norp02aa1n02x5               g188(.a(new_n283), .b(new_n277), .o1(\s[28] ));
  norb02aa1n02x5               g189(.a(new_n271), .b(new_n276), .out0(new_n285));
  oai012aa1n02x5               g190(.a(new_n285), .b(new_n281), .c(new_n278), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[28] ), .b(\b[27] ), .c(new_n274), .carry(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  aoi012aa1n02x5               g193(.a(new_n288), .b(new_n286), .c(new_n287), .o1(new_n289));
  aobi12aa1n02x5               g194(.a(new_n285), .b(new_n270), .c(new_n268), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n287), .c(new_n288), .out0(new_n291));
  norp02aa1n02x5               g196(.a(new_n289), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g198(.a(new_n271), .b(new_n288), .c(new_n276), .out0(new_n294));
  oai012aa1n02x5               g199(.a(new_n294), .b(new_n281), .c(new_n278), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n287), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n02x5               g203(.a(new_n294), .b(new_n270), .c(new_n268), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n02x5               g205(.a(new_n298), .b(new_n300), .o1(\s[30] ));
  norb02aa1n02x5               g206(.a(new_n294), .b(new_n297), .out0(new_n302));
  aobi12aa1n02x5               g207(.a(new_n302), .b(new_n270), .c(new_n268), .out0(new_n303));
  oao003aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  nano22aa1n02x4               g210(.a(new_n303), .b(new_n304), .c(new_n305), .out0(new_n306));
  oai012aa1n02x5               g211(.a(new_n302), .b(new_n281), .c(new_n278), .o1(new_n307));
  aoi012aa1n02x5               g212(.a(new_n305), .b(new_n307), .c(new_n304), .o1(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n306), .o1(\s[31] ));
  xnrb03aa1n02x5               g214(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g215(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g217(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g218(.a(new_n115), .b(new_n108), .out0(new_n314));
  oai012aa1n02x5               g219(.a(new_n314), .b(\b[4] ), .c(\a[5] ), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g221(.a(new_n109), .b(new_n315), .out0(new_n317));
  oaib12aa1n02x5               g222(.a(new_n317), .b(\a[6] ), .c(new_n117), .out0(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanb02aa1n02x5               g224(.a(new_n110), .b(new_n111), .out0(new_n320));
  nanb02aa1n02x5               g225(.a(new_n112), .b(new_n113), .out0(new_n321));
  nanb02aa1n02x5               g226(.a(new_n321), .b(new_n318), .out0(new_n322));
  aoi012aa1n02x5               g227(.a(new_n320), .b(new_n322), .c(new_n120), .o1(new_n323));
  nanp03aa1n02x5               g228(.a(new_n322), .b(new_n320), .c(new_n120), .o1(new_n324));
  norb02aa1n02x5               g229(.a(new_n324), .b(new_n323), .out0(\s[8] ));
  xnbna2aa1n03x5               g230(.a(new_n123), .b(new_n141), .c(new_n142), .out0(\s[9] ));
endmodule


